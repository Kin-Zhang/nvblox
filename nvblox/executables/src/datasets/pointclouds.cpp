/*
Copyright 2022 NVIDIA CORPORATION

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
#include "nvblox/datasets/pointclouds.h"

#include <glog/logging.h>

#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
// #include <filesystem>

#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "nvblox/utils/timing.h"

namespace nvblox {
namespace datasets {
namespace pointclouds {
// CHANGE Point Type Here!!! If you want to use XYZI, change to pcl::PointXYZI
// typedef pcl::PointXYZ PointType;
typedef pcl::PointXYZ PointType;

DepthImage depthImageFromPointcloud(const Eigen::MatrixX3f& pointcloud,
                                    const Lidar& lidar) {
  DepthImage depth_image(lidar.num_elevation_divisions(),
                         lidar.num_azimuth_divisions(), MemoryType::kUnified);
  for (int idx = 0; idx < pointcloud.rows(); idx++) {
    const Vector3f p_C = pointcloud.row(idx);
    Index2D u_C;
    lidar.project(p_C, &u_C);
    depth_image(u_C.y(), u_C.x()) = p_C.norm();
  }
  return depth_image;
}
void ReadConfig(const std::string &config_path, Config* config) {
  CHECK_NOTNULL(config);
  config->num_azimuth_divisions = 1024;
  config->num_elevation_divisions = 16;
  config->vertical_fov_rad = 30.0f * M_PI / 180.0;
  // ReadYaml(config_path, config);
  LOG(INFO) << "Loading config from: " << config_path;
  // PrintConfig(*config);
}

Dataloader::Dataloader(const Config config){
  LOG(INFO) << "Loading data from " << config.data_path;
  
  for (const auto &entry : fs::directory_iterator(
        fs::path(config.data_path) / "pcd")) {
      filenames_.push_back(entry.path().string());
  }
  std::sort(filenames_.begin(), filenames_.end());
  total_frame_number_ = filenames_.size();
}

bool Dataloader::loadNext(Transform* T_L_C_ptr, PointCloud& pointcloud_ptr){
  if (frame_number_ >= total_frame_number_){
    LOG(INFO) << "All data loaded";
    return false;
  }
  LOG(INFO) << "Loading frame " << frame_number_;
  pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
  if (pcl::io::loadPCDFile<PointType>(filenames_[frame_number_], *cloud) == -1) //* load the file
  {
    LOG(WARNING) << "Couldn't read file " << filenames_[frame_number_];
    return false;
  }
  pointcloud_ptr = cloud->getMatrixXfMap(3, 4, 0).transpose();
  frame_number_++;
  return true;
}

Integration::Integration(){

}

void Integration::integrateFrame(const PointCloud &pointcloud, const Transform* T_L_C_ptr){
  
}
}  // namespace pointclouds
}  // namespace datasets
}  // namespace nvblox
