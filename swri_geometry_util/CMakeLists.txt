cmake_minimum_required(VERSION 3.0.2)

project(swri_geometry_util)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR})

find_package(GEOS REQUIRED)

find_package(OpenCV REQUIRED core)

find_package(Eigen3 REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    tf
  DEPENDS
    OpenCV
)

include_directories(include)
include_directories(SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${GEOS_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/cubic_spline.cpp
  src/geometry_util.cpp
  src/intersection.cpp
  src/polygon.cpp
)
target_compile_definitions(${PROJECT_NAME} PUBLIC "-DUSE_UNSTABLE_GEOS_CPP_API")
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${GEOS_LIBRARIES}
  opencv_core
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_intersection test/test_intersection.cpp)
  target_link_libraries(test_intersection ${PROJECT_NAME})

  ament_add_gtest(test_geometry_util test/test_geometry_util.cpp)
  target_link_libraries(test_geometry_util ${PROJECT_NAME})
endif()

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(cv_bridge)
ament_export_dependencies(tf2)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}
  ${GEOS_LIBRARIES})

ament_package()
