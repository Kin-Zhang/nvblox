# Datasets library
add_library(nvblox_datasets SHARED
    src/datasets/3dmatch.cpp
    src/datasets/image_loader.cpp
    src/datasets/replica.cpp
    src/fuser.cpp
)
target_include_directories(nvblox_datasets PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(nvblox_datasets nvblox_lib)
set_target_properties(nvblox_datasets PROPERTIES CUDA_SEPARABLE_COMPILATION ON)


# 3Dmatch executable
add_executable(fuse_3dmatch
    src/fuse_3dmatch.cpp
)
target_link_libraries(fuse_3dmatch
    nvblox_lib nvblox_datasets
)
set_target_properties(fuse_3dmatch PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

# LiDAR executable
find_package(PCL REQUIRED)
add_library(pointcloud SHARED
    src/datasets/pointclouds.cpp
)
target_include_directories(pointcloud PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
# issue met here and solved: https://stackoverflow.com/a/52137618/9281669
target_link_libraries(pointcloud nvblox_lib ${PCL_LIBRARIES} stdc++fs)
add_executable(fuse_lidar
    src/fuse_lidar.cpp
)
target_link_libraries(fuse_lidar
    nvblox_lib pointcloud ${PCL_LIBRARIES}
)
set_target_properties(fuse_lidar PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

# Replica executable
add_executable(fuse_replica
    src/fuse_replica.cpp
)
target_link_libraries(fuse_replica
    nvblox_lib nvblox_datasets
)
set_target_properties(fuse_replica PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
