/**
 * Copyright (C) 2023-now, RPL, KTH Royal Institute of Technology
 * MIT License
 * @author Kin ZHANG (https://kin-zhang.github.io/)
 * @date: 2023-10-15 17:33
 * @details: No ROS version, speed up the process
 *
 * Input: TODO
 * Output: TODO
*/

#pragma once

#include <memory>
#include <string>

#include "nvblox/core/types.h"
#include "nvblox/core/common_names.h"
#include "nvblox/core/image.h"
#include "nvblox/core/layer.h"
#include "nvblox/core/lidar.h"
#include "nvblox/core/types.h"
#include "nvblox/integrators/projective_tsdf_integrator.h"
#include "nvblox/integrators/view_calculator.h"
#include "nvblox/io/csv.h"
#include "nvblox/io/mesh_io.h"
#include "nvblox/io/pointcloud_io.h"
#include "nvblox/mesh/mesh_integrator.h"

namespace nvblox {
namespace datasets {
namespace pointclouds {

using PointCloud = Eigen::MatrixX3f;
struct Config {
  std::string data_path = "/home/kin/data/DynamicMap_OnlyData/semindoor";

  // global parameters
  float max_range = 40.0f;
  float min_range = 0.1f;

  // lidar parameters
  int num_azimuth_divisions = 1024;
  int num_elevation_divisions = 32; // normally based on how many channel you have in lidar, VLP-16 here
  float vertical_fov_rad = 30.0f * M_PI / 180.0;
  
  // nvblox parameters
  float voxel_size = 0.1f;
  int truncation_distance_vox = 4;
  float truncation_distance = truncation_distance_vox * voxel_size;
};
DepthImage depthImageFromPointcloud(const Eigen::MatrixX3f& pointcloud,
                                    const Lidar& lidar);

void ReadConfig(const std::string &config_path, Config* config);

class Dataloader{
 public:
  Dataloader(const Config config);
  bool loadNext(Transform* T_L_C_ptr, PointCloud& pointcloud_ptr);
 protected:
  int total_frame_number_ = -1;
  int frame_number_ = 0;
  std::vector<std::string> filenames_;
};

class Integration{
 public:
  Integration();
  void integrateFrame(const PointCloud &pointcloud, const Transform* T_L_C_ptr);
};

}  // namespace threedmatch
}  // namespace datasets
}  // namespace nvblox
