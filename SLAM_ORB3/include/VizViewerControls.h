#ifndef ORB_SLAM3_VIZ_VIEWER_CONTROLS_H
#define ORB_SLAM3_VIZ_VIEWER_CONTROLS_H

#include <opencv2/viz.hpp>
#include <algorithm>

namespace ORB_SLAM3
{
class VizViewerControls
{
public:
    explicit VizViewerControls(const cv::Affine3d &initialPose): mInitialPose(initialPose), mResetRequested(true), mTopFollow(false), mHintShown(false), mHintDirty(true), mTopHeight(std::max(5.0, cv::norm(initialPose.translation()) * 3.0))
    {
    }

    static void KeyboardCallback(const cv::viz::KeyboardEvent &event, void* cookie)
    {
        if(event.action != cv::viz::KeyboardEvent::KEY_DOWN)
            return;
        VizViewerControls* controls = static_cast<VizViewerControls*>(cookie);
        if(event.code == 'r' || event.code == 'R' || event.symbol == "r" || event.symbol == "R")
            controls->RequestReset();
        else if(event.code == 't' || event.code == 'T' || event.symbol == "t" || event.symbol == "T")
            controls->RequestTopView();
    }

    static void MouseCallback(const cv::viz::MouseEvent &event, void* cookie)
    {
        VizViewerControls* controls = static_cast<VizViewerControls*>(cookie);
        if(event.type == cv::viz::MouseEvent::MouseButtonPress && event.button == cv::viz::MouseEvent::LeftButton && event.pointer.x >= 10 && event.pointer.x <= 295 && event.pointer.y >= 8 && event.pointer.y <= 42)
        {
            controls->RequestTopView();
            return;
        }
        if(event.type == cv::viz::MouseEvent::MouseButtonPress || event.type == cv::viz::MouseEvent::MouseScrollUp || event.type == cv::viz::MouseEvent::MouseScrollDown)
            controls->StopFollowing();
    }

    void RequestReset()
    {
        mResetRequested = true;
        StopFollowing();
    }

    void RequestTopView()
    {
        mResetRequested = false;
        mTopFollow = !mTopFollow;
        mHintDirty = true;
    }

    bool IsTopFollowing() const
    {
        return mTopFollow;
    }

    // Call after event dispatch: VTK's default R action also runs in OnChar.
    void ApplyReset(cv::viz::Viz3d &window)
    {
        if(mResetRequested)
        {
            window.setViewerPose(mInitialPose);
            mResetRequested = false;
        }
    }

    void ApplyView(cv::viz::Viz3d &window, const cv::Affine3d &cameraPose, const bool gravityAligned)
    {
        ApplyReset(window);
        if(mTopFollow)
        {
            // IMU initialization aligns world +Z with up. Before that, and for
            // visual-only maps, use the initial camera convention: world -Y up.
            const cv::Vec3d up = gravityAligned ? cv::Vec3d(0, 0, 1) : cv::Vec3d(0, -1, 0);
            const cv::Vec3d downInImage = gravityAligned ? cv::Vec3d(0, 1, 0) : cv::Vec3d(0, 0, -1);
            const cv::Vec3d center = cameraPose.translation();
            window.setViewerPose(cv::viz::makeCameraPose(center + up * mTopHeight, center, downInImage));
        }
        UpdateHint(window);
    }

    void UpdateHint(cv::viz::Viz3d &window)
    {
        if(!mHintShown)
        {
            window.showWidget("viewer/top_hint", cv::viz::WText("T = View from TOP", cv::Point(15, 15), 18, cv::viz::Color(150, 70, 20)));
            mHintShown = true;
        }
        if(mHintDirty)
        {
            cv::viz::WText hint = window.getWidget("viewer/top_hint").cast<cv::viz::WText>();
            hint.setText(mTopFollow ? "T = View from TOP [ON]" : "T = View from TOP");
            mHintDirty = false;
        }
    }

private:
    void StopFollowing()
    {
        mTopFollow = false;
        mHintDirty = true;
    }

    cv::Affine3d mInitialPose;
    bool mResetRequested;
    bool mTopFollow;
    bool mHintShown;
    bool mHintDirty;
    double mTopHeight;
};
}

#endif
