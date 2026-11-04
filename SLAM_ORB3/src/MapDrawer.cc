/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include "MapDrawer.h"
#include "VizViewerProfile.h"
#include "VizBatchLines.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <map>
#include <mutex>

namespace ORB_SLAM3
{


MapDrawer::MapDrawer(Atlas* pAtlas, const string &strSettingPath, Settings* settings):mpAtlas(pAtlas)
{
    if(settings){
        newParameterLoader(settings);
    }
    else{
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
        bool is_correct = ParseViewerParamFile(fSettings);

        if(!is_correct)
        {
            std::cerr << "**ERROR in the config file, the format is not correct**" << std::endl;
            try
            {
                throw -1;
            }
            catch(exception &e)
            {

            }
        }
    }
}

void MapDrawer::newParameterLoader(Settings *settings) {
    mKeyFrameSize = settings->keyFrameSize();
    mKeyFrameLineWidth = settings->keyFrameLineWidth();
    mGraphLineWidth = settings->graphLineWidth();
    mPointSize = settings->pointSize();
    mCameraSize = settings->cameraSize();
    mCameraLineWidth  = settings->cameraLineWidth();
}

bool MapDrawer::ParseViewerParamFile(cv::FileStorage &fSettings)
{
    bool b_miss_params = false;

    cv::FileNode node = fSettings["Viewer.KeyFrameSize"];
    if(!node.empty())
    {
        mKeyFrameSize = node.real();
    }
    else
    {
        std::cerr << "*Viewer.KeyFrameSize parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.KeyFrameLineWidth"];
    if(!node.empty())
    {
        mKeyFrameLineWidth = node.real();
    }
    else
    {
        std::cerr << "*Viewer.KeyFrameLineWidth parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.GraphLineWidth"];
    if(!node.empty())
    {
        mGraphLineWidth = node.real();
    }
    else
    {
        std::cerr << "*Viewer.GraphLineWidth parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.PointSize"];
    if(!node.empty())
    {
        mPointSize = node.real();
    }
    else
    {
        std::cerr << "*Viewer.PointSize parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.CameraSize"];
    if(!node.empty())
    {
        mCameraSize = node.real();
    }
    else
    {
        std::cerr << "*Viewer.CameraSize parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.CameraLineWidth"];
    if(!node.empty())
    {
        mCameraLineWidth = node.real();
    }
    else
    {
        std::cerr << "*Viewer.CameraLineWidth parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    return !b_miss_params;
}

namespace
{
cv::Affine3d ToVizPose(const Sophus::SE3f &pose)
{
    const Eigen::Matrix4f matrix = pose.matrix();
    cv::Matx44d converted;
    for(int row = 0; row < 4; ++row)
    {
        for(int col = 0; col < 4; ++col)
            converted(row, col) = matrix(row, col);
    }
    return cv::Affine3d(converted);
}

cv::Point3d CameraCenter(const cv::Affine3d &pose)
{
    const cv::Vec3d translation = pose.translation();
    return cv::Point3d(translation[0], translation[1], translation[2]);
}

cv::viz::WPolyLine CameraFrustum(const float width, const cv::viz::Color &color)
{
    const float height = width * 0.75f;
    const float depth = width * 0.6f;
    const cv::Point3f origin(0, 0, 0), a(width, height, depth), b(width, -height, depth), c(-width, -height, depth), d(-width, height, depth);
    // Walk the original eight edges; repeated edges introduce no extra geometry.
    const std::vector<cv::Point3f> vertices = {origin, a, b, origin, c, d, origin, a, d, c, b};
    return cv::viz::WPolyLine(vertices, color);
}

struct CameraSnapshot
{
    std::string id;
    cv::Affine3d pose;
    cv::viz::Color color;
    float lineWidth;
};

using Edge = std::pair<cv::Point3d, cv::Point3d>;
}

void MapDrawer::UpdateViewer(cv::viz::Viz3d &window, const bool drawGraph)
{
    static VizViewerProfile profile("map", {"lock_wait", "snapshot", "widgets"});
    const auto profileBegin = VizViewerProfile::Clock::now();
    double lockWait = 0;
    // Current pose stays responsive even when the map snapshot must be skipped.
    if(!mWidgetIds.count("camera/current"))
    {
        window.showWidget("camera/current", CameraFrustum(mCameraSize, cv::viz::Color::green()), GetCurrentCameraPose());
        window.setRenderingProperty("camera/current", cv::viz::LINE_WIDTH, mCameraLineWidth);
        mWidgetIds.insert("camera/current");
    }
    else
        window.setWidgetPose("camera/current", GetCurrentCameraPose());
    std::vector<cv::Point3f> points, referencePoints;
    std::vector<CameraSnapshot> cameras;
    std::vector<Edge> graph, inertialGraph;
    Map* activeMap = nullptr;
    std::vector<Map*> maps;
    // Unlike GetCurrentMap(), this read-only accessor never creates a SLAM map.
    mpAtlas->GetViewerMaps(activeMap, maps);
    for(Map* map : maps)
    {
        // Tracking/optimization use this mutex. Reset also stops the viewer before clearing maps.
        // In this repository culled maps/keyframes/points are retained until shutdown.
        const auto lockBegin = VizViewerProfile::Clock::now();
        std::unique_lock<std::mutex> mapLock(map->mMutexMapUpdate, std::try_to_lock);
        if(!mapLock.owns_lock())
            return; // Keep the previous complete scene; never stall image/event processing.
        lockWait += VizViewerProfile::Milliseconds(lockBegin, VizViewerProfile::Clock::now());
        const bool active = map == activeMap;
        if(active)
        {
            const std::vector<MapPoint*> reference = map->GetReferenceMapPoints();
            const std::set<MapPoint*> referenceSet(reference.begin(), reference.end());
            for(MapPoint* point : map->GetAllMapPoints())
            {
                if(!point || point->isBad() || referenceSet.count(point))
                    continue;
                const Eigen::Vector3f position = point->GetWorldPos();
                if(!position.allFinite())
                    continue;
                points.emplace_back(position.x(), position.y(), position.z());
            }
            // The original viewer drew the reference set independently of GetAllMapPoints().
            // Keep that exact selection, including references temporarily outside the map set.
            for(MapPoint* point : referenceSet)
            {
                if(!point || point->isBad())
                    continue;
                const Eigen::Vector3f position = point->GetWorldPos();
                if(position.allFinite())
                    referencePoints.emplace_back(position.x(), position.y(), position.z());
            }
        }

        std::vector<KeyFrame*> keyframes = map->GetAllKeyFrames();
        std::map<KeyFrame*, cv::Affine3d> poses;
        for(KeyFrame* keyframe : keyframes)
        {
            if(!keyframe || keyframe->isBad())
                continue;
            const cv::Affine3d pose = ToVizPose(keyframe->GetPoseInverse());
            poses.emplace(keyframe, pose);
            const bool root = keyframe->GetParent() == nullptr;
            const unsigned int colorIndex = keyframe->mnOriginMapId % 6;
            const cv::viz::Color color = root ? cv::viz::Color::red() : (active ? cv::viz::Color::blue() : cv::viz::Color(255 * mfFrameColors[colorIndex][2], 255 * mfFrameColors[colorIndex][1], 255 * mfFrameColors[colorIndex][0]));
            cameras.push_back({"keyframe/" + std::to_string(keyframe->mnId), pose, color, root ? mKeyFrameLineWidth * 5 : mKeyFrameLineWidth});
        }

        if(!active)
            continue;
        for(const auto &entry : poses)
        {
            KeyFrame* keyframe = entry.first;
            const cv::Point3d center = CameraCenter(entry.second);
            if(drawGraph)
            {
                for(KeyFrame* neighbor : keyframe->GetCovisiblesByWeight(100))
                {
                    const auto found = poses.find(neighbor);
                    if(found != poses.end() && neighbor->mnId > keyframe->mnId)
                        graph.emplace_back(center, CameraCenter(found->second));
                }
                const auto parent = poses.find(keyframe->GetParent());
                if(parent != poses.end())
                    graph.emplace_back(center, CameraCenter(parent->second));
                for(KeyFrame* neighbor : keyframe->GetLoopEdges())
                {
                    const auto found = poses.find(neighbor);
                    if(found != poses.end() && neighbor->mnId > keyframe->mnId)
                        graph.emplace_back(center, CameraCenter(found->second));
                }
            }
        }
        if(map->isImuInitialized())
        {
            // Read the existing IMU links through the synchronized accessor.
            for(const auto &entry : poses)
            {
                const auto next = poses.find(entry.first->GetNextKeyFrame());
                if(next != poses.end())
                    inertialGraph.emplace_back(CameraCenter(entry.second), CameraCenter(next->second));
            }
        }
    }

    const auto snapshotEnd = VizViewerProfile::Clock::now();
    // All VTK work happens after releasing SLAM locks. Only value copies reach Viz.
    std::set<std::string> visible;
    const auto showCloud = [&](const std::string &id, const std::vector<cv::Point3f> &cloud, const cv::viz::Color &color)
    {
        if(cloud.empty())
            return;
        window.showWidget(id, cv::viz::WCloud(cloud, color));
        window.setRenderingProperty(id, cv::viz::POINT_SIZE, mPointSize);
        visible.insert(id);
    };
    showCloud("map/points", points, cv::viz::Color::black());
    showCloud("map/reference_points", referencePoints, cv::viz::Color::red());
    for(const CameraSnapshot &camera : cameras)
    {
        if(!mWidgetIds.count(camera.id))
            window.showWidget(camera.id, CameraFrustum(mKeyFrameSize, camera.color), camera.pose);
        else
            window.setWidgetPose(camera.id, camera.pose);
        // A map can become inactive after relocalization, changing its camera color.
        window.getWidget(camera.id).cast<cv::viz::Widget3D>().setColor(camera.color);
        window.setRenderingProperty(camera.id, cv::viz::LINE_WIDTH, camera.lineWidth);
        visible.insert(camera.id);
    }
    const auto showEdges = [&](const std::string &id, const std::vector<Edge> &edges, const cv::viz::Color &color)
    {
        if(edges.empty())
            return;
        UpdateVizBatchLines(window, id, edges, color, mGraphLineWidth, mWidgetIds.count(id) != 0);
        visible.insert(id);
    };
    showEdges("map/graph", graph, cv::viz::Color::green());
    showEdges("map/inertial_graph", inertialGraph, cv::viz::Color::red());
    visible.insert("camera/current");
    for(const std::string &id : mWidgetIds)
    {
        if(!visible.count(id))
            window.removeWidget(id);
    }
    mWidgetIds.swap(visible);
    profile.Add({lockWait, VizViewerProfile::Milliseconds(profileBegin, snapshotEnd), VizViewerProfile::Milliseconds(snapshotEnd, VizViewerProfile::Clock::now())});
}

void MapDrawer::SetCurrentCameraPose(const Sophus::SE3f &Tcw)
{
    std::unique_lock<std::mutex> lock(mMutexCamera);
    // Tracking supplies world-to-camera; widgets need camera-to-world, exactly once.
    mCameraPose = Tcw.inverse();
}

cv::Affine3d MapDrawer::GetCurrentCameraPose()
{
    std::unique_lock<std::mutex> lock(mMutexCamera);
    return ToVizPose(mCameraPose);
}

} // namespace ORB_SLAM3
