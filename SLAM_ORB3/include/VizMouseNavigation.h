#ifndef ORB_SLAM3_VIZ_MOUSE_NAVIGATION_H
#define ORB_SLAM3_VIZ_MOUSE_NAVIGATION_H

#include <opencv2/core.hpp>

namespace ORB_SLAM3
{
// Mouse navigation with free orbit, tf=0.01 and zf=0.1.
// Matrices here use OpenGL camera axes (+X right, +Y up, -Z forward).
class VizMouseNavigation
{
public:
    enum ButtonMask { Left = 1, Middle = 2, Right = 4, WheelUp = 8, WheelDown = 16, Shift = 1 << 16, Ctrl = 1 << 17, Alt = 1 << 18 };

    VizMouseNavigation();
    cv::Matx44d Button(int button, int x, int y, bool pressed, int state, float depth, const cv::Matx44d &projection, const cv::Vec4i &viewport);
    cv::Matx44d Motion(int x, int y, int state, float depth, const cv::Matx44d &projection, const cv::Vec4i &viewport);

private:
    cv::Vec3d Unproject(int x, int y, double depth, const cv::Matx44d &projection, const cv::Vec4i &viewport) const;
    void Select(int x, int y, float depth, const cv::Matx44d &projection, const cv::Vec4i &viewport);

    double mLastDepth;
    cv::Point2f mLastPosition;
    cv::Vec3d mRotationCenter;
};
}

#endif
