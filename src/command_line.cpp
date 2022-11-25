// mergescan.cpp : Defines the entry point for the application.
//

#include <cstdio>
#include <iostream>
#include <open3d/camera/PinholeCameraIntrinsic.h>
#include <open3d/io/ImageIO.h>
#include <geometry/Geometry.h>
#include <geometry/Image.h>
#include <geometry/RGBDImage.h>
#include <geometry/PointCloud.h>
#include "ThreadPool.h"

using namespace std;
using namespace open3d;

std::shared_ptr<geometry::PointCloud> createPointCloud(const std::string& path) {
    geometry::Image img;
    io::ReadImageFromPNG(path, img);
    //auto rgbd = geometry::RGBDImage::CreateFromColorAndDepth(img, img);
    camera::PinholeCameraIntrinsic intrinsics;
    return geometry::PointCloud::CreateFromDepthImage(img, intrinsics);
}

int main()
{
    std::vector<JoinableFuture<std::shared_ptr<geometry::PointCloud>>> result;
    ThreadPool threadPool(8);
    for(int i = 0; i < 119; i++) {
        char path[256];
        sprintf(path, "test_to_process/frame_%05d/depth__%05d.png", i, i);
        auto future = threadPool.submit([path = std::string(path)]() -> std::shared_ptr<geometry::PointCloud> {
            std::cerr << "Loading " << path << std::endl;
            return createPointCloud(path);
        });
        
        auto future2 = threadPool.submit([future]() {
            auto pointCloud = future.get();
            return get<0>(pointCloud->RemoveStatisticalOutliers(10, 1, false));
        }, future);
        result.emplace_back(std::move(future2));
    }
    for(auto& pointCloudFuture: result) {
        auto pointCloud = pointCloudFuture.get();
        std::cerr << "size = " << pointCloud->points_.size() << std::endl;
    }
    return 0;
}
