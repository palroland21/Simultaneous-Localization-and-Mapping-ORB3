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

#ifndef ORB3_LIVE_MONITOR_H
#define ORB3_LIVE_MONITOR_H

#include <System.h>
#include <algorithm>
#include <iomanip>
#include <iostream>
#include "Stereo-Inertial/LiveGroundTruth.h"

namespace orb3_live {

inline void PrintEvaluation(const orb3_live::GroundTruth::Result& error, const char* label)
{
    std::cout << std::fixed << std::setprecision(3) << label;
    if (error.valid())
    {
        if (error.positionValid) std::cout << " PositionError=" << error.positionError << "m";
        else std::cout << " PositionError=N/A";
        std::cout << " | ATE_RMSE=" << error.rmse << "m";
    }
    else
        std::cout << " PositionError=N/A | ATE_RMSE=N/A | Reason=" << error.status;
    std::cout << " | Samples=" << error.samples << std::endl;
}

// Keep the same fields for both IMU modes; an unused CAM1 reports 0.0 Hz.
inline void PrintMonitor(ORB_SLAM3::System& SLAM, GroundTruth& evaluation, const std::vector<double>& cameraTimes, const std::vector<double>& imuTimes, double tframe, double processingSeconds, size_t imuSamples, bool firstFrame, bool stereo, double& lastMonitorTime)
{
    const double elapsedTime = tframe - cameraTimes[0];
    Sophus::SE3f Twb;
    unsigned long mapId = 0, originId = 0;
    bool imuInitialized = false;
    const bool bodyTracked = SLAM.GetCurrentBodyPose(Twb, mapId, originId, imuInitialized);

    if (firstFrame || elapsedTime - lastMonitorTime >= 1.0)
    {
        lastMonitorTime = elapsedTime;
        orb3_live::GroundTruth::Result error;
        if (imuInitialized)
        {
            error = evaluation.evaluate(SLAM.GetOptimizedBodyTrajectory(), cameraTimes.front(), tframe);
            if (!bodyTracked) error.positionValid = false;
        }
        else error.status = bodyTracked ? "WAIT_IMU" : "WAIT_TRACKING";

        const int trackingState = SLAM.GetTrackingState();
        const std::vector<ORB_SLAM3::MapPoint*> trackedPoints = SLAM.GetTrackedMapPoints();
        const size_t trackedCount = std::count_if(trackedPoints.begin(), trackedPoints.end(), [](ORB_SLAM3::MapPoint* point) {
            return point != nullptr;
        });

        std::string stateName = "UNKNOWN";
        if (trackingState == 1) stateName = "INITIALIZING";
        else if (trackingState == 2) stateName = "OK";
        else if (trackingState == 3) stateName = "RECENTLY_LOST";
        else if (trackingState == 4) stateName = "LOST";
        else if (trackingState == 5) stateName = "OK_KLT";

        const double windowStart = tframe - 1.0;
        const auto camBegin = std::lower_bound(cameraTimes.cbegin(), cameraTimes.cend(), windowStart);
        const auto camEnd = std::upper_bound(cameraTimes.cbegin(), cameraTimes.cend(), tframe);
        const auto imuBegin = std::lower_bound(imuTimes.cbegin(), imuTimes.cend(), windowStart);
        const auto imuEnd = std::upper_bound(imuTimes.cbegin(), imuTimes.cend(), tframe);

        const auto calculateHz = [](std::vector<double>::const_iterator begin, std::vector<double>::const_iterator end) -> double {
            const auto count = std::distance(begin, end);
            if (count < 2) return 0.0;
            const double duration = *(end - 1) - *begin;
            return duration > 0.0 ? static_cast<double>(count - 1) / duration : 0.0;
        };

        const double camHz = calculateHz(camBegin, camEnd);
        const double imuHz = calculateHz(imuBegin, imuEnd);

        std::cout << std::fixed << std::setprecision(3);
        std::cout << "[ORB3 LIVE] t=" << elapsedTime << "s";
        std::cout << " | State=" << stateName;
        std::cout << " | TrackedMP=" << trackedCount;
        std::cout << " | IMU_Samples=" << imuSamples;
        std::cout << " | Processing=" << processingSeconds * 1000.0 << "ms" << std::endl;

        if (bodyTracked && imuInitialized)
        {
            const Eigen::Vector3f position = Twb.translation();
            const Eigen::Quaternionf orientation = Twb.unit_quaternion();

            std::cout << "[ORB3 LIVE] p_BinW=(" << position.x() << "," << position.y() << "," << position.z() << ")m";
            std::cout << std::setprecision(1);
            std::cout << " | CAM0=" << camHz << "Hz | CAM1=" << (stereo ? camHz : 0.0) << "Hz | IMU=" << imuHz << "Hz" << std::endl;
            std::cout << std::setprecision(3);
            std::cout << "[ORB3 LIVE] q_BtoW_xyzw=(" << orientation.x() << "," << orientation.y() << "," << orientation.z() << "," << orientation.w() << ")" << std::endl;
        }
        else
        {
            std::cout << std::setprecision(1);
            std::cout << "[ORB3 LIVE] Position=N/A";
            std::cout << " | CAM0=" << camHz << "Hz | CAM1=" << (stereo ? camHz : 0.0) << "Hz | IMU=" << imuHz << "Hz" << std::endl;
            std::cout << std::setprecision(3);
        }

        PrintEvaluation(error, "[ORB3 EVAL]");
        std::cout << std::endl;
    }
}

} // namespace orb3_live

#endif
