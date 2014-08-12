cmake_minimum_required(VERSION 2.8.3)
project(ransac_primitives)

set(CMAKE_CXX_FLAGS "-O4 -Wall -fPIC -std=c++0x ${CMAKE_CXX_FLAGS}")
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(catkin REQUIRED)
# PCL
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# OpenCV
find_package(OpenCV REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES primitive_core primitive_shapes
   CATKIN_DEPENDS 
   DEPENDS OpenCV PCL 
)

set(CORE_HEADERS
include/primitive_params.h
include/primitive_octree.h include/primitive_octree.hpp
include/primitive_extractor.h include/primitive_extractor.hpp
include/primitive_visualizer.h include/primitive_visualizer.hpp)

add_library(primitive_core src/primitive_leaf.cpp src/base_primitive.cpp src/primitive_core.cpp ${CORE_HEADERS})
add_library(primitive_shapes src/plane_primitive.cpp src/sphere_primitive.cpp src/cylinder_primitive.cpp)
add_executable(test_extraction src/test_extraction.cpp)

target_link_libraries(primitive_core ${OpenCV_LIBS} ${PCL_LIBRARIES})
target_link_libraries(primitive_shapes primitive_core)
target_link_libraries(test_extraction primitive_shapes primitive_core ${PCL_LIBRARIES})
