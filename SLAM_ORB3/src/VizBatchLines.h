#ifndef ORB_SLAM3_VIZ_BATCH_LINES_H
#define ORB_SLAM3_VIZ_BATCH_LINES_H

#include <opencv2/viz.hpp>
#include <utility>
#include <vector>

namespace ORB_SLAM3
{
void UpdateVizBatchLines(cv::viz::Viz3d &window, const std::string &id, const std::vector<std::pair<cv::Point3d, cv::Point3d>> &edges, const cv::viz::Color &color, const double lineWidth, const bool exists);
}

#endif
