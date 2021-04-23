#include <open3d/Open3D.h>

#include <Eigen/Core>
#include <memory>
#include <string>

namespace o3d = open3d;

// Custom Visualizer Wrapper, copy python implementation
void CustomDrawGeometries(
    const std::vector<std::shared_ptr<const o3d::geometry::Geometry>>
        &geometry_ptrs,
    Eigen::Vector3d *lookat = nullptr,
    Eigen::Vector3d *up = nullptr,
    Eigen::Vector3d *front = nullptr,
    double *zoom = nullptr,
    double point_size = 1.0,
    const std::string window_name = "PlaneSegmentation C++",
    const int width = 1920,
    const int height = 1080,
    const int left = 50,
    const int top = 50) {
    // Create a new Window
    o3d::visualization::Visualizer vis;
    vis.CreateVisualizerWindow(window_name, width, height, left, top, true);

    // Add the provided geometries to the canvas
    for (const auto &geom : geometry_ptrs) {
        vis.AddGeometry(geom);
    }

    // Change the render options
    o3d::visualization::RenderOption &render_options = vis.GetRenderOption();
    render_options.SetPointSize(point_size);

    // Change the viewpoint of the camera
    o3d::visualization::ViewControl &view_control = vis.GetViewControl();
    if (lookat != nullptr) {
        view_control.SetLookat(*lookat);
    }
    if (up != nullptr) {
        view_control.SetUp(*up);
    }
    if (front != nullptr) {
        view_control.SetFront(*front);
    }
    if (zoom != nullptr) {
        view_control.SetZoom(*zoom);
    }

    vis.Run();
    vis.DestroyVisualizerWindow();
}
