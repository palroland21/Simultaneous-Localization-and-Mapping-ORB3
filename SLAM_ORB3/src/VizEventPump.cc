#include "VizEventPump.h"
#include "VizMouseNavigation.h"
#include <opencv2/viz/widget_accessor.hpp>
#include <vtkProp.h>
#include <vtkCommand.h>
#include <vtkCamera.h>
#include <vtkMatrix4x4.h>
#include <array>
#include <algorithm>
#include <vtkInteractorStyle.h>
#include <vtkNew.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>

namespace ORB_SLAM3
{
namespace
{
vtkRenderer* RendererForWidget(cv::viz::Viz3d &window, const std::string &anchorId)
{
    const auto prop = cv::viz::WidgetAccessor::getProp(window.getWidget(anchorId));
    for(int i = 0; i < prop->GetNumberOfConsumers(); ++i)
    {
        if(vtkRenderer* renderer = vtkRenderer::SafeDownCast(prop->GetConsumer(i)))
            return renderer;
    }
    CV_Error(cv::Error::StsError, "Viz widget has no renderer");
}

cv::Matx44d CopyMatrix(vtkMatrix4x4* matrix)
{
    cv::Matx44d copy;
    for(int row = 0; row < 4; ++row)
    {
        for(int col = 0; col < 4; ++col)
            copy(row, col) = matrix->GetElement(row, col);
    }
    return copy;
}

class MouseNavigationCommand : public vtkCommand
{
public:
    static MouseNavigationCommand* New()
    {
        return new MouseNavigationCommand;
    }

    cv::viz::Viz3d::MouseCallback callback = nullptr;
    void* cookie = nullptr;

    void Execute(vtkObject* source, unsigned long eventId, void*) override
    {
        vtkInteractorStyle* style = vtkInteractorStyle::SafeDownCast(source);
        if(!style || !style->GetInteractor())
            return;
        vtkRenderWindowInteractor* interactor = style->GetInteractor();
        const int x = interactor->GetEventPosition()[0], y = interactor->GetEventPosition()[1];
        style->FindPokedRenderer(x, y);
        vtkRenderer* renderer = style->GetCurrentRenderer();
        if(!renderer)
            return;
        int modifiers = 0, vizModifiers = 0;
        if(interactor->GetShiftKey())
        {
            modifiers |= VizMouseNavigation::Shift;
            vizModifiers |= cv::viz::KeyboardEvent::SHIFT;
        }
        if(interactor->GetControlKey())
        {
            modifiers |= VizMouseNavigation::Ctrl;
            vizModifiers |= cv::viz::KeyboardEvent::CTRL;
        }
        if(interactor->GetAltKey())
        {
            modifiers |= VizMouseNavigation::Alt;
            vizModifiers |= cv::viz::KeyboardEvent::ALT;
        }
        // Pick from the already displayed frame, before native focus handlers
        // can trigger an extra render (same timing as the original mouse handler).
        vtkCamera* camera = renderer->GetActiveCamera();
        const cv::Matx44d projection = CopyMatrix(camera->GetProjectionTransformMatrix(renderer->GetTiledAspectRatio(), -1, 1));
        const int* origin = renderer->GetOrigin();
        const int* size = renderer->GetSize();
        const cv::Vec4i viewport(origin[0], origin[1], size[0], size[1]);
        float depth = 1.0f;
        const bool pressEvent = eventId == vtkCommand::LeftButtonPressEvent || eventId == vtkCommand::MiddleButtonPressEvent || eventId == vtkCommand::RightButtonPressEvent || eventId == vtkCommand::MouseWheelForwardEvent || eventId == vtkCommand::MouseWheelBackwardEvent;
        if(pressEvent || ((modifiers & VizMouseNavigation::Ctrl) && (modifiers & VizMouseNavigation::Shift)))
            depth = PickDepth(renderer, x, y);
        int button = 0;
        bool pressed = true;
        cv::viz::MouseEvent::MouseButton vizButton = cv::viz::MouseEvent::NoButton;
        cv::viz::MouseEvent::Type vizType = cv::viz::MouseEvent::MouseButtonPress;
        switch(eventId)
        {
        case vtkCommand::LeftButtonPressEvent:
            button = VizMouseNavigation::Left;
            vizButton = cv::viz::MouseEvent::LeftButton;
            style->OnLeftButtonDown();
            style->StopState();
            break;
        case vtkCommand::RightButtonPressEvent:
            button = VizMouseNavigation::Right;
            vizButton = cv::viz::MouseEvent::RightButton;
            style->OnRightButtonDown();
            style->StopState();
            break;
        case vtkCommand::MiddleButtonPressEvent:
            button = VizMouseNavigation::Middle;
            vizButton = cv::viz::MouseEvent::MiddleButton;
            style->OnMiddleButtonDown();
            style->StopState();
            break;
        case vtkCommand::LeftButtonReleaseEvent:
            button = VizMouseNavigation::Left;
            vizButton = cv::viz::MouseEvent::LeftButton;
            pressed = false;
            style->OnLeftButtonUp();
            break;
        case vtkCommand::RightButtonReleaseEvent:
            button = VizMouseNavigation::Right;
            vizButton = cv::viz::MouseEvent::RightButton;
            pressed = false;
            style->OnRightButtonUp();
            break;
        case vtkCommand::MiddleButtonReleaseEvent:
            button = VizMouseNavigation::Middle;
            vizButton = cv::viz::MouseEvent::MiddleButton;
            pressed = false;
            style->OnMiddleButtonUp();
            break;
        case vtkCommand::MouseWheelForwardEvent:
            button = VizMouseNavigation::WheelUp;
            vizButton = cv::viz::MouseEvent::VScroll;
            vizType = cv::viz::MouseEvent::MouseScrollUp;
            break;
        case vtkCommand::MouseWheelBackwardEvent:
            button = VizMouseNavigation::WheelDown;
            vizButton = cv::viz::MouseEvent::VScroll;
            vizType = cv::viz::MouseEvent::MouseScrollDown;
            break;
        case vtkCommand::MouseMoveEvent:
            vizType = cv::viz::MouseEvent::MouseMove;
            break;
        default:
            return;
        }
        if(!pressed)
            vizType = cv::viz::MouseEvent::MouseButtonRelease;
        if(callback)
            callback(cv::viz::MouseEvent(vizType, vizButton, cv::Point(x, y), vizModifiers), cookie);
        if(button == VizMouseNavigation::Left || button == VizMouseNavigation::Middle || button == VizMouseNavigation::Right)
            mButtons = pressed ? mButtons | button : mButtons & ~button;
        cv::Matx44d delta = cv::Matx44d::eye();
        if(button)
            delta = mNavigation.Button(button, x, y, pressed, mButtons | modifiers, depth, projection, viewport);
        else if(mButtons || modifiers)
            delta = mNavigation.Motion(x, y, mButtons | modifiers, depth, projection, viewport);
        if(cv::norm(cv::Mat(delta), cv::Mat(cv::Matx44d::eye())) == 0)
            return;
        const cv::Matx44d modelView = CopyMatrix(camera->GetModelViewTransformMatrix());
        const cv::Matx44d inverse = (delta * modelView).inv();
        const double distance = camera->GetDistance();
        camera->SetPosition(inverse(0, 3), inverse(1, 3), inverse(2, 3));
        camera->SetFocalPoint(inverse(0, 3) - inverse(0, 2) * distance, inverse(1, 3) - inverse(1, 2) * distance, inverse(2, 3) - inverse(2, 2) * distance);
        camera->SetViewUp(inverse(0, 1), inverse(1, 1), inverse(2, 1));
    }

private:
    float PickDepth(vtkRenderer* renderer, int x, int y)
    {
        vtkRenderWindow* window = renderer->GetRenderWindow();
        const int* size = window->GetSize();
        const int x0 = std::max(0, x - 8), y0 = std::max(0, y - 8);
        const int x1 = std::min(size[0] - 1, x + 8), y1 = std::min(size[1] - 1, y + 8);
        std::array<float, 17 * 17> depths;
        depths.fill(1.0f);
        if(x1 < x0 || y1 < y0 || !window->GetZbufferData(x0, y0, x1, y1, depths.data()))
            return 1.0f;
        return *std::min_element(depths.begin(), depths.begin() + (x1 - x0 + 1) * (y1 - y0 + 1));
    }

    VizMouseNavigation mNavigation;
    int mButtons = 0;
};
}

bool ProcessVizEvents(cv::viz::Viz3d &window, const std::string &anchorId, cv::viz::Viz3d::MouseCallback mouseCallback, void* mouseCookie)
{
    if(window.wasStopped())
        return false;
    vtkRenderer* renderer = RendererForWidget(window, anchorId);
    vtkRenderWindow* renderWindow = renderer->GetRenderWindow();
    if(!renderWindow->GetInteractor())
    {
        // Bootstrap Viz's native interactor and its close/keyboard observers once.
        // Subsequent frames never enter Interactor::Start(), which can remain
        // inside its event loop while a mouse button is held.
        window.spinOnce(1, false);
        renderWindow = renderer->GetRenderWindow();
        if(window.wasStopped())
            return false;
    }
    // Closing Viz may release its interactor during ProcessEvents(). Keep a
    // local reference alive until native event dispatch has returned.
    vtkSmartPointer<vtkRenderWindowInteractor> interactor = renderWindow->GetInteractor();
    vtkInteractorStyle* style = vtkInteractorStyle::SafeDownCast(interactor->GetInteractorStyle());
    if(style && !style->HasObserver(vtkCommand::LeftButtonPressEvent))
    {
        vtkNew<MouseNavigationCommand> navigation;
        navigation->callback = mouseCallback;
        navigation->cookie = mouseCookie;
        style->HandleObserversOn();
        const unsigned long events[] = {vtkCommand::LeftButtonPressEvent, vtkCommand::LeftButtonReleaseEvent, vtkCommand::RightButtonPressEvent, vtkCommand::RightButtonReleaseEvent, vtkCommand::MiddleButtonPressEvent, vtkCommand::MiddleButtonReleaseEvent, vtkCommand::MouseMoveEvent, vtkCommand::MouseWheelForwardEvent, vtkCommand::MouseWheelBackwardEvent};
        for(unsigned long event : events)
            style->AddObserver(event, navigation);
    }
    interactor->SetDone(false);
    interactor->ProcessEvents();
    if(interactor->GetDone())
        return false;
    renderWindow->Render();
    return true;
}

void SetVizCameraProjection(cv::viz::Viz3d &window, const std::string &anchorId, const cv::viz::Camera &intrinsics)
{
    cv::Matx44d projection;
    intrinsics.computeProjectionMatrix(projection);
    vtkNew<vtkMatrix4x4> matrix;
    for(int row = 0; row < 4; ++row)
    {
        for(int col = 0; col < 4; ++col)
            matrix->SetElement(row, col, projection(row, col));
    }
    vtkCamera* camera = RendererForWidget(window, anchorId)->GetActiveCamera();
    // Match the original viewer's fixed projection and clip planes. OpenCV setCamera()
    // uses a user transform whose effective depth mapping changes when VTK
    // resets its native clip range; that also changes blank-space drag speed.
    camera->SetUserTransform(nullptr);
    camera->SetExplicitProjectionTransformMatrix(matrix);
    camera->UseExplicitProjectionTransformMatrixOn();
    window.setWindowSize(intrinsics.getWindowSize());
}
}
