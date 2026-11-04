#ifndef ORB_SLAM3_VIZ_EVENT_PUMP_H
#define ORB_SLAM3_VIZ_EVENT_PUMP_H

#include <opencv2/viz.hpp>

namespace ORB_SLAM3
{
bool ProcessVizEvents(cv::viz::Viz3d &window, const std::string &anchorId, cv::viz::Viz3d::MouseCallback mouseCallback = nullptr, void* mouseCookie = nullptr);
void SetVizCameraProjection(cv::viz::Viz3d &window, const std::string &anchorId, const cv::viz::Camera &intrinsics);
}

#endif
