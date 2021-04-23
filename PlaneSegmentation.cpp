#include <open3d/Open3D.h>

#include <iostream>
#include <memory>
#include <string>

using namespace open3d;

std::shared_ptr<geometry::PointCloud> ReadPointCloud(
    const std::string& filename) {
    auto cloud_ptr = std::make_shared<geometry::PointCloud>();
    if (io::ReadPointCloud(filename, *cloud_ptr)) {
        utility::LogInfo("Successfully read {}\n", filename);
    } else {
        utility::LogError("Failed to read {}\n\n", filename);
    }
    return cloud_ptr;
}

int main(int argc, char* argv[]) {
    utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);
    if (argc < 2) {
        utility::LogInfo("Open3D: {}", OPEN3D_VERSION);
        utility::LogInfo("Usage:");
        utility::LogInfo("{} [filename]", argv[0]);
        return 0;
    }

    auto pcd = ReadPointCloud(argv[1]);
    visualization::DrawGeometries({pcd}, "PointCloud");

    return 0;
}
