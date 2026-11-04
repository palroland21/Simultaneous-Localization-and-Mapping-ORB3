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


#include "Viewer.h"
#include "VizViewerControls.h"
#include "VizEventPump.h"
#include "VizViewerProfile.h"
#include <opencv2/viz.hpp>
#include <algorithm>
#include <chrono>

#include <mutex>

namespace ORB_SLAM3
{

Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath, Settings* settings):
    both(false), mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),mpMapDrawer(pMapDrawer), mpTracker(pTracking),
    mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false)
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

    mbStopTrack = false;
}

void Viewer::newParameterLoader(Settings *settings) {
    mImageViewerScale = 1.f;

    float fps = settings->fps();
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    cv::Size imSize = settings->newImSize();
    mImageHeight = imSize.height;
    mImageWidth = imSize.width;

    mImageViewerScale = settings->imageViewerScale();
    mViewpointX = settings->viewPointX();
    mViewpointY = settings->viewPointY();
    mViewpointZ = settings->viewPointZ();
    mViewpointF = settings->viewPointF();
}

bool Viewer::ParseViewerParamFile(cv::FileStorage &fSettings)
{
    bool b_miss_params = false;
    mImageViewerScale = 1.f;

    float fps = fSettings["Camera.fps"];
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    cv::FileNode node = fSettings["Camera.width"];
    if(!node.empty())
    {
        mImageWidth = node.real();
    }
    else
    {
        std::cerr << "*Camera.width parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Camera.height"];
    if(!node.empty())
    {
        mImageHeight = node.real();
    }
    else
    {
        std::cerr << "*Camera.height parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.imageViewScale"];
    if(!node.empty())
    {
        mImageViewerScale = node.real();
    }

    node = fSettings["Viewer.ViewpointX"];
    if(!node.empty())
    {
        mViewpointX = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointX parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.ViewpointY"];
    if(!node.empty())
    {
        mViewpointY = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointY parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.ViewpointZ"];
    if(!node.empty())
    {
        mViewpointZ = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointZ parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.ViewpointF"];
    if(!node.empty())
    {
        mViewpointF = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointF parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    return !b_miss_params;
}

void Viewer::Run()
{
    {
        std::unique_lock<std::mutex> lock(mMutexFinish);
        mbFinished = false;
    }
    {
        std::unique_lock<std::mutex> lock(mMutexStop);
        mbStopped = false;
    }

    const std::string frameWindow = "ORB-SLAM3: Current Frame";
    try
    {
        cv::viz::Viz3d window("ORB-SLAM3: Map Viewer");
        window.setWindowSize(cv::Size(1024, 768));
        window.setBackgroundColor(cv::viz::Color::white());
        cv::viz::Camera initialCamera(mViewpointF, mViewpointF, 512, 389, cv::Size(1024, 768));
        initialCamera.setClip(cv::Vec2d(0.1, 1000));
        // Fix the projection/clip planes once; navigation changes only the pose.
        bool projectionInitialized = false;
        // Viz uses camera +Y down: y_dir=+Y reproduces the original viewer's world up=(0,-1,0).
        const cv::Affine3d initialPose = cv::viz::makeCameraPose(cv::Vec3d(mViewpointX, mViewpointY, mViewpointZ), cv::Vec3d(0, 0, 0), cv::Vec3d(0, 1, 0));
        VizViewerControls controls(initialPose);
        window.registerKeyboardCallback(VizViewerControls::KeyboardCallback, &controls);
        controls.UpdateHint(window);
        cv::namedWindow(frameWindow);
        const float trackedImageScale = mpTracker->GetImageScale();
        cout << "Starting the OpenCV Viz Viewer (R: initial view, T: follow from TOP)" << endl;
        VizViewerProfile profile("loop", {"map", "image", "viz", "highgui", "total"});
        bool gravityAligned = false;
        while(!CheckFinish() && !window.wasStopped())
        {
            const auto frameStart = std::chrono::steady_clock::now();
            // Reset/ResetActiveMap waits for this acknowledgement before clearing maps.
            Stop();
            const auto mapBegin = VizViewerProfile::Clock::now();
            auto mapEnd = mapBegin;
            if(!isStopped())
            {
                mpMapDrawer->UpdateViewer(window, true);
                mapEnd = VizViewerProfile::Clock::now();
                cv::Mat toShow;
                cv::Mat im = mpFrameDrawer->DrawFrame(trackedImageScale);
                if(both)
                {
                    cv::Mat imRight = mpFrameDrawer->DrawRightFrame(trackedImageScale);
                    if(imRight.rows != im.rows)
                        cv::resize(imRight, imRight, cv::Size(imRight.cols * im.rows / imRight.rows, im.rows));
                    cv::hconcat(im, imRight, toShow);
                }
                else
                    toShow = im;
                const float displayScale = both ? std::min(mImageViewerScale, 1000.f / toShow.cols) : mImageViewerScale;
                if(displayScale != 1.f)
                {
                    const int width = std::max(1, static_cast<int>(toShow.cols * displayScale));
                    const int height = std::max(1, static_cast<int>(toShow.rows * displayScale));
                    cv::resize(toShow, toShow, cv::Size(width, height), 0, 0, displayScale < 1.f ? cv::INTER_AREA : cv::INTER_LINEAR);
                }
                cv::imshow(frameWindow, toShow);
            }

            // Keep both event loops alive, including while SLAM has paused the viewer.
            const auto imageEnd = VizViewerProfile::Clock::now();
            if(!ProcessVizEvents(window, "viewer/top_hint", VizViewerControls::MouseCallback, &controls))
                break;
            const auto vizEnd = VizViewerProfile::Clock::now();
            const double elapsed = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - frameStart).count();
            const int key = cv::waitKey(std::max(1, static_cast<int>(mT - elapsed)));
            if(key == 'r' || key == 'R')
                controls.RequestReset();
            else if(key == 't' || key == 'T')
                controls.RequestTopView();
            if(window.wasStopped() || cv::getWindowProperty(frameWindow, cv::WND_PROP_VISIBLE) < 1)
                break;
            // Initialize the native projection after Viz has created its window.
            if(!projectionInitialized)
            {
                SetVizCameraProjection(window, "viewer/top_hint", initialCamera);
                projectionInitialized = true;
            }
            // VTK handles R in OnChar too, AFTER the keyboard callback. Defer our
            // reset until both event pumps finish so its default cannot override us.
            if(controls.IsTopFollowing() && !isStopped())
            {
                Map* activeMap = nullptr;
                std::vector<Map*> maps;
                mpMapDrawer->mpAtlas->GetViewerMaps(activeMap, maps);
                gravityAligned = activeMap && activeMap->isImuInitialized();
            }
            controls.ApplyView(window, mpMapDrawer->GetCurrentCameraPose(), gravityAligned);
            const auto loopEnd = VizViewerProfile::Clock::now();
            profile.Add({VizViewerProfile::Milliseconds(mapBegin, mapEnd), VizViewerProfile::Milliseconds(mapEnd, imageEnd), VizViewerProfile::Milliseconds(imageEnd, vizEnd), VizViewerProfile::Milliseconds(vizEnd, loopEnd), VizViewerProfile::Milliseconds(frameStart, loopEnd)});
        }
        window.close();
        cv::destroyWindow(frameWindow);
    }
    catch(const std::exception &error)
    {
        std::cerr << "OpenCV Viz viewer stopped: " << error.what() << std::endl;
    }
    SetFinish();
}

void Viewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Viewer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish()
{
    unique_lock<mutex> stopLock(mMutexStop);
    unique_lock<mutex> finishLock(mMutexFinish);
    mbStopped = true;
    mbFinished = true;
}

bool Viewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void Viewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool Viewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool Viewer::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void Viewer::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> finishLock(mMutexFinish);
    if(!mbFinished)
        mbStopped = false;
}

/*void Viewer::SetTrackingPause()
{
    mbStopTrack = true;
}*/

}
