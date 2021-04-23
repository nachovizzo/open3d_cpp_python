#include <open3d/Open3D.h>

#include <Eigen/Core>
#include <iostream>
#include <memory>
#include <string>
#include <tuple>

#include "CustomDrawGeometries.hpp"
namespace o3d = open3d;

// Custom Visualization Viewpoint for this particular scan
Eigen::Vector3d lookat{2.53, 1.12, -5.31};
Eigen::Vector3d up{0.49, 0.05, 0.87};
Eigen::Vector3d front{-0.86, -0.13, 0.49};
double zoom{0.1};

auto SegmentAndVisualizePointCloud(
    std::shared_ptr<o3d::geometry::PointCloud> pcd,
    // Default params taken from Open3D implementation
    const double distance_threshold = 0.01,
    const int ransac_n = 3,
    const int num_iterations = 100) {
    auto [plane_model, plane_points] =
        pcd->SegmentPlane(distance_threshold, ransac_n, num_iterations);

    auto plane_cloud = pcd->SelectByIndex(plane_points);
    plane_cloud->PaintUniformColor({1, 0, 0});

    auto rest_cloud = pcd->SelectByIndex(plane_points, true);
    rest_cloud->PaintUniformColor({0, 0, 1});

    CustomDrawGeometries(
        {plane_cloud, rest_cloud}, &lookat, &up, &front, &zoom);

    return std::make_tuple(plane_cloud, rest_cloud);
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
    auto pcd = std::make_shared<o3d::geometry::PointCloud>();
    o3d::io::ReadPointCloud(argv[1], *pcd);

    // Initial inspection of the data
    o3d::utility::LogInfo("Visualizing KITTI Sequence 07 scan");
    CustomDrawGeometries({pcd}, &lookat, &up, &front, &zoom);

    // PlaneSegmentation ---- < copy from this line for your application
    o3d::utility::LogInfo("Running segmentation with default parameters...");
    SegmentAndVisualizePointCloud(pcd);

    // Obtain the arguments from the Python
    const double distance_threshold = std::stod(argv[2]);
    const int ransac_n = std::stoi(argv[3]);
    const int num_iterations = std::stoi(argv[4]);
    o3d::utility::LogInfo("Running segmentation with custom parameters...");
    SegmentAndVisualizePointCloud(
        pcd, distance_threshold, ransac_n, num_iterations);

    // Benchmark the segmentation algorithm
    {
        o3d::utility::ScopeTimer t{"SegmentPlane"};
        auto [plane_model, plane_points] =
            pcd->SegmentPlane(distance_threshold, ransac_n, num_iterations);
    }

    return 0;
}
