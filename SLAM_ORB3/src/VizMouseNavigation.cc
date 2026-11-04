/*
 * Copyright (c) 2013 Steven Lovegrove
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include "VizMouseNavigation.h"
#include <cmath>

namespace ORB_SLAM3
{
namespace
{
cv::Matx44d Transform(const cv::Matx33d &rotation, const cv::Vec3d &translation)
{
    cv::Matx44d result = cv::Matx44d::eye();
    for(int row = 0; row < 3; ++row)
    {
        for(int col = 0; col < 3; ++col)
            result(row, col) = rotation(row, col);
        result(row, 3) = translation[row];
    }
    return result;
}

cv::Matx33d Rotation(double x, double y, double z)
{
    const double sx = std::sin(x), sy = std::sin(y), sz = std::sin(z);
    const double cx = std::cos(x), cy = std::cos(y), cz = std::cos(z);
    return cv::Matx33d(cy * cz, -cy * sz, sy, sx * sy * cz + cx * sz, -sx * sy * sz + cx * cz, -sx * cy, -cx * sy * cz + sx * sz, cx * sy * sz + sx * cz, cx * cy);
}
}

VizMouseNavigation::VizMouseNavigation(): mLastDepth(0.8), mLastPosition(0, 0), mRotationCenter(0, 0, 0)
{
}

cv::Vec3d VizMouseNavigation::Unproject(int x, int y, double depth, const cv::Matx44d &projection, const cv::Vec4i &viewport) const
{
    const cv::Vec4d clip(2.0 * (x - viewport[0]) / viewport[2] - 1.0, 2.0 * (y - viewport[1]) / viewport[3] - 1.0, 2.0 * depth - 1.0, 1.0);
    const cv::Vec4d camera = projection.inv() * clip;
    return cv::Vec3d(camera[0], camera[1], camera[2]) / camera[3];
}

void VizMouseNavigation::Select(int x, int y, float depth, const cv::Matx44d &projection, const cv::Vec4i &viewport)
{
    if(depth == 1.0f)
        depth = static_cast<float>(mLastDepth);
    if(depth != 1.0f)
    {
        mLastDepth = depth;
        mRotationCenter = Unproject(x, y, depth, projection, viewport);
    }
}

cv::Matx44d VizMouseNavigation::Button(int button, int x, int y, bool pressed, int state, float depth, const cv::Matx44d &projection, const cv::Vec4i &viewport)
{
    mLastPosition = cv::Point2f(x, y);
    if(!pressed)
        return cv::Matx44d::eye();
    Select(x, y, depth, projection, viewport);
    if(button == WheelUp || button == WheelDown)
    {
        cv::Vec3d translation(0, 0, button == WheelUp ? 1.0 : -1.0);
        if(!(state & Right) && mRotationCenter != cv::Vec3d(0, 0, 0))
            translation = mRotationCenter * (button == WheelUp ? -static_cast<double>(0.1f) : static_cast<double>(0.1f));
        return Transform(cv::Matx33d::eye(), translation);
    }
    return cv::Matx44d::eye();
}

cv::Matx44d VizMouseNavigation::Motion(int x, int y, int state, float depth, const cv::Matx44d &projection, const cv::Vec4i &viewport)
{
    const float dx = static_cast<float>(x) - mLastPosition.x;
    const float dy = static_cast<float>(y) - mLastPosition.y;
    if((state & Ctrl) && (state & Shift))
        Select(x, y, depth, projection, viewport);
    cv::Matx44d result = cv::Matx44d::eye();
    if(dx * dx + dy * dy < 50.0f * 50.0f)
    {
        if(state == Left)
        {
            cv::Vec3d translation(-10.0 * dx * static_cast<double>(0.01f), 10.0 * dy * static_cast<double>(0.01f), 0);
            if(mLastDepth != 1.0)
            {
                const cv::Vec3d next = Unproject(x, y, mLastDepth, projection, viewport);
                translation = cv::Vec3d(next[0] - mRotationCenter[0], next[1] - mRotationCenter[1], 0);
                mRotationCenter = next;
            }
            result = Transform(cv::Matx33d::eye(), translation);
        }
        else if(state == Right || state == Middle || state == (Left | Right))
        {
            double pitch = -0.01 * dy;
            double yaw = state == Middle ? 0.01 * dx * -projection(3, 2) : -0.01 * dx;
            double roll = 0;
            if(state == Right && projection(3, 2) <= 0)
                yaw *= -1;
            if(state == (Left | Right))
            {
                pitch = yaw = 0;
                roll = dx * 0.01;
            }
            const cv::Matx33d rotation = Rotation(pitch, yaw, roll);
            const cv::Vec3d translation = state == Middle ? cv::Vec3d(0, 0, 0) : mRotationCenter - rotation * mRotationCenter;
            result = Transform(rotation, translation);
        }
    }
    mLastPosition = cv::Point2f(x, y);
    return result;
}
}
