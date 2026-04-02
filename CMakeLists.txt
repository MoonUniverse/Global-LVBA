cmake_minimum_required(VERSION 3.8)
project(global_lvba)

if(EXISTS "/usr/bin/python3")
  set(Python3_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH "Python 3 interpreter for ROS2/ament" FORCE)
  set(Python_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH "Python interpreter for ROS2/ament" FORCE)
endif()

set(CMAKE_BUILD_TYPE "Release")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native -mtune=native -funroll-loops")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

include(ProcessorCount)
ProcessorCount(N)
message(STATUS "Processor count: ${N}")

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(OpenMP REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)           
find_package(GLUT REQUIRED)          
find_package(Sophus REQUIRED)
find_package(CURL REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
if(EXISTS "/usr/local/lib/cmake/Ceres/CeresConfig.cmake")
  set(Ceres_DIR "/usr/local/lib/cmake/Ceres" CACHE PATH "Preferred Ceres config" FORCE)
endif()
find_package(Ceres REQUIRED)
find_package(SQLite3 REQUIRED)

set(Sophus_LIBRARIES libSophus.so)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/SiftGPU/src/SiftGPU
)

set(SIFTGPU_LIBS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/SiftGPU/build/src/SiftGPU/libsiftgpu.a
)

if(NOT EXISTS ${SIFTGPU_LIBS})
  message(FATAL_ERROR "Missing SiftGPU static library at ${SIFTGPU_LIBS}. Build the submodule before building this package.")
endif()

add_library(lvba_system src/lvba_system.cpp)
add_library(lvba_io src/dataset_io.cpp)

target_link_libraries(lvba_system
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${pcl_conversions_LIBRARIES}
  Ceres::ceres
  Eigen3::Eigen
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Sophus_LIBRARIES}
  ${SIFTGPU_LIBS}
  GLEW::GLEW
  GLUT::GLUT
  OpenGL::GL
  CURL::libcurl
  SQLite::SQLite3
)

target_link_libraries(lvba_io
  ${rclcpp_LIBRARIES}
  ${Sophus_LIBRARIES}
)

add_executable(lidar_visual_ba src/main.cpp)
target_link_libraries(lidar_visual_ba
  ${rclcpp_LIBRARIES}
  lvba_system
  lvba_io
)

install(
  TARGETS lidar_visual_ba lvba_system lvba_io
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch rviz_cfg DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)

ament_package()
