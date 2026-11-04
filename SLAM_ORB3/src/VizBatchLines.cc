#include "VizBatchLines.h"
#include <opencv2/viz/widget_accessor.hpp>
#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkNew.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>

namespace ORB_SLAM3
{
void UpdateVizBatchLines(cv::viz::Viz3d &window, const std::string &id, const std::vector<std::pair<cv::Point3d, cv::Point3d>> &edges, const cv::viz::Color &color, const double lineWidth, const bool exists)
{
    vtkNew<vtkPoints> points;
    points->SetNumberOfPoints(edges.size() * 2);
    vtkNew<vtkCellArray> lines;
    lines->AllocateEstimate(edges.size(), 2);
    vtkIdType index = 0;
    for(const auto &edge : edges)
    {
        points->SetPoint(index, edge.first.x, edge.first.y, edge.first.z);
        points->SetPoint(index + 1, edge.second.x, edge.second.y, edge.second.z);
        const vtkIdType endpoints[2] = {index, index + 1};
        lines->InsertNextCell(2, endpoints);
        index += 2;
    }
    vtkNew<vtkPolyData> data;
    data->SetPoints(points);
    data->SetLines(lines);
    if(exists)
    {
        cv::viz::Widget widget = window.getWidget(id);
        vtkActor* actor = vtkActor::SafeDownCast(cv::viz::WidgetAccessor::getProp(widget));
        vtkPolyDataMapper::SafeDownCast(actor->GetMapper())->SetInputData(data);
        return;
    }
    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputData(data);
    mapper->ScalarVisibilityOff();
    vtkNew<vtkActor> actor;
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(color[2] / 255.0, color[1] / 255.0, color[0] / 255.0);
    actor->GetProperty()->SetLineWidth(lineWidth);
    actor->GetProperty()->SetOpacity(0.6);
    actor->GetProperty()->LightingOff();
    cv::viz::Widget3D widget;
    cv::viz::WidgetAccessor::setProp(widget, actor);
    window.showWidget(id, widget);
}
}
