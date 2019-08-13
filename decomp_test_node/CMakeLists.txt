cmake_minimum_required(VERSION 2.8.3)
project(decomp_test_node)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-deprecated-declarations")

find_package(catkin_simple REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(decomp_util REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL 1.2 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS} ${DECOMP_UTIL_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

catkin_simple()

cs_add_executable(test_path_decomp_2d src/test_path_decomp_2d.cpp)
cs_add_executable(test_path_decomp_3d src/test_path_decomp_3d.cpp)
cs_add_executable(test_seed_decomp src/test_seed_decomp.cpp)
cs_add_executable(pcd_read_test src/pcd_read.cpp)
cs_add_executable(png2pcd_test src/png2pcd.cpp)

cs_install()
cs_export()

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(pcd_read_test ${PCL_LIBRARIES})
target_link_libraries(test_path_decomp_2d ${PCL_LIBRARIES})
target_link_libraries(png2pcd_test ${PCL_LIBRARIES})
