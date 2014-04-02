cmake_minimum_required(VERSION 2.8.3)
project(ransac_primitives)

set(CMAKE_CXX_FLAGS "-O4 -Wall -fPIC -std=c++0x")
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(catkin REQUIRED)
# PCL
#set(PCL_DIR "/usr/share/pcl-1.7")
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# OpenCV
#include_directories(/opt/ros/hydro/include)
#link_directories(/opt/ros/hydro/lib)
find_package(OpenCV REQUIRED)

catkin_package(
   INCLUDE_DIRS src
   LIBRARIES basic_primitives primitive_octree primitive_extractor
   CATKIN_DEPENDS 
   DEPENDS OpenCV PCL 
)

# MAIN LIBRARY
#add_library(base_primitive src/base_primitive.cpp)
#add_library(primitive_leaf src/primitive_leaf.cpp)
#add_library(primitive_octree src/primitive_octree.cpp src/base_primitive.cpp)
#add_library(primitive_visualizer src/primitive_visualizer.cpp)
#add_library(primitive_extractor src/primitive_extractor.cpp)
#add_library(plane_primitive src/plane_primitive.cpp)
#add_library(sphere_primitive src/sphere_primitive.cpp)
#add_library(cylinder_primitive src/cylinder_primitive.cpp)
#add_executable(test_primitives src/test_primitives.cpp)
#add_executable(test_find_point src/test_find_point.cpp)
#add_executable(test_sphere_components src/test_sphere_components.cpp)

add_library(primitive_octree src/primitive_leaf.cpp src/primitive_octree.cpp src/base_primitive.cpp)
add_library(primitive_extractor src/primitive_visualizer.cpp src/primitive_extractor.cpp)
add_library(basic_primitives src/plane_primitive.cpp src/sphere_primitive.cpp src/cylinder_primitive.cpp)
#add_executable(test_primitives src/test_primitives.cpp)
add_executable(test_find_point src/test_find_point.cpp)

#target_link_libraries(base_primitive primitive_octree ${OpenCV_LIBS} ${PCL_LIBRARIES}) # opencv_core opencv_imgproc opencv_highgui
#target_link_libraries(plane_primitive primitive_octree ${PCL_LIBRARIES})
#target_link_libraries(sphere_primitive primitive_octree ${PCL_LIBRARIES})
#target_link_libraries(cylinder_primitive primitive_octree ${PCL_LIBRARIES})
#target_link_libraries(primitive_leaf ${PCL_LIBRARIES})
#target_link_libraries(primitive_octree primitive_leaf ${OpenCV_LIBS} ${PCL_LIBRARIES})
#target_link_libraries(primitive_extractor primitive_visualizer primitive_octree ${PCL_LIBRARIES})
#target_link_libraries(primitive_visualizer ${PCL_LIBRARIES})
#target_link_libraries(test_primitives primitive_extractor plane_primitive primitive_visualizer ${PCL_LIBRARIES})
#target_link_libraries(test_find_point primitive_extractor plane_primitive sphere_primitive cylinder_primitive primitive_visualizer ${PCL_LIBRARIES})
#target_link_libraries(test_sphere_components sphere_primitive)

target_link_libraries(basic_primitives primitive_octree ${PCL_LIBRARIES})
target_link_libraries(primitive_octree ${OpenCV_LIBS} ${PCL_LIBRARIES})
target_link_libraries(primitive_extractor primitive_octree ${PCL_LIBRARIES})
#target_link_libraries(test_primitives primitive_extractor basic_primitives ${PCL_LIBRARIES})
target_link_libraries(test_find_point primitive_extractor basic_primitives ${PCL_LIBRARIES})
