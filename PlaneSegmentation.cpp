#include <open3d/Open3D.h>

#include <Eigen/Core>
#include <iostream>
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

std::shared_ptr<o3d::geometry::PointCloud> ReadPointCloud(
    const std::string &filename) {
    auto cloud_ptr = std::make_shared<o3d::geometry::PointCloud>();
    if (!o3d::io::ReadPointCloud(filename, *cloud_ptr)) {
        o3d::utility::LogError("Failed to read {}", filename);
    }
    return cloud_ptr;
}

int main(int argc, char *argv[]) {
    o3d::utility::SetVerbosityLevel(o3d::utility::VerbosityLevel::Debug);
    if (argc < 2) {
        o3d::utility::LogInfo("Open3D: {}", OPEN3D_VERSION);
        o3d::utility::LogInfo("Usage:");
        o3d::utility::LogInfo("{} [filename] [segmentation_params]", argv[0]);
        return 0;
    }

    // Read the point cloud
    auto pcd = ReadPointCloud(argv[1]);

    // Custom Visualization Viewpoint for this particular scan
    Eigen::Vector3d lookat{2.53, 1.12, -5.31};
    Eigen::Vector3d up{0.49, 0.05, 0.87};
    Eigen::Vector3d front{-0.86, -0.13, 0.49};
    double zoom{0.1};

    // Obtain the arguments from the Python
    const double distance_threshold = std::stod(argv[2]);
    const int ransac_n = std::stoi(argv[3]);
    const int num_iterations = std::stoi(argv[4]);

    // Initial inspection of the data
    o3d::utility::LogInfo("Visualizing KITTI Sequence 07 scan");
    CustomDrawGeometries({pcd}, &lookat, &up, &front, &zoom);

    // PlaneSegmentation ---- < copy from this line for your application
    o3d::utility::LogInfo("Running segmentation algorithm...");
    auto [plane_model, plane_points] =
        pcd->SegmentPlane(distance_threshold, ransac_n, num_iterations);

    auto plane_cloud = pcd->SelectByIndex(plane_points);
    plane_cloud->PaintUniformColor({1, 0, 0});

    auto rest_cloud = pcd->SelectByIndex(plane_points, true);
    rest_cloud->PaintUniformColor({0, 0, 1});

    o3d::utility::LogInfo("Visualizing results of the plane segmentation");
    CustomDrawGeometries(
        {plane_cloud, rest_cloud}, &lookat, &up, &front, &zoom);

    // Benchmark the segmentation algorithm
    {
        o3d::utility::ScopeTimer t{"SegmentPlane"};
        auto [plane_model, plane_points] =
            pcd->SegmentPlane(distance_threshold, ransac_n, num_iterations);
    }

    return 0;
}
