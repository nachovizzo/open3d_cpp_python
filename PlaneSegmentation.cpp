#include <open3d/Open3D.h>
#include <open3d/visualization/utility/DrawGeometry.h>

#include <Eigen/Core>
#include <iostream>
#include <memory>
#include <string>

namespace o3d = open3d;

// Custom Visualizer Wrapper
bool DrawGeometries(
    const std::vector<std::shared_ptr<const o3d::geometry::Geometry>>
        &geometry_ptrs) {
    const std::string window_name = "PlaneSegmentation C++";
    const int width = 1920;
    const int height = 1080;
    const int left = 50;
    const int top = 50;
    const bool point_show_normal = false;
    const bool mesh_show_wireframe = false;
    const bool mesh_show_back_face = false;
    Eigen::Vector3d lookat{-0.86, -0.13, 0.49};
    Eigen::Vector3d up{2.53, 1.12, -5.31};
    Eigen::Vector3d front{0.49, 0.05, 0.87};
    double zoom = 0.1;
    return o3d::visualization::DrawGeometries({geometry_ptrs},
                                              window_name,
                                              width,
                                              height,
                                              left,
                                              top,
                                              point_show_normal,
                                              mesh_show_wireframe,
                                              mesh_show_back_face,
                                              &lookat,
                                              &up,
                                              &front,
                                              &zoom);
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
    o3d::utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);
    if (argc < 2) {
        o3d::utility::LogInfo("Open3D: {}", OPEN3D_VERSION);
        o3d::utility::LogInfo("Usage:");
        o3d::utility::LogInfo("{} [filename]", argv[0]);
        return 0;
    }

    // Read the point cloud
    auto pcd = ReadPointCloud(argv[1]);

    // Initial inspection of the data
    o3d::utility::LogInfo("KITTI Sequence 07 scan:");
    DrawGeometries({pcd});

    // PlaneSegmentation

    return 0;
}
