
##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.4.6)
add_definitions(-std=c++0x)

##############################################################################
# Ros Initialisation
##############################################################################

project(top_kinect2)
find_package(catkin REQUIRED COMPONENTS roscpp qt_build std_msgs sensor_msgs stereo_msgs tf)

catkin_package()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

##############################################################################
# Qt Environment
##############################################################################

#include(qt_build qt-ros)
rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here
ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/TopKinect/*.hpp)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(top_kinect2 ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(top_kinect2 ${QT_LIBRARIES} ${catkin_LIBRARIES})
