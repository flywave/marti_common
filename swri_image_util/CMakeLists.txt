cmake_minimum_required(VERSION 3.0.2)
project(swri_image_util)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(swri_geometry_util REQUIRED)
find_package(swri_math_util REQUIRED)
find_package(swri_opencv_util REQUIRED)
find_package(swri_roscpp REQUIRED)
find_package(tf2 REQUIRED)

find_package(OpenCV REQUIRED COMPONENTS core)

find_package(Boost REQUIRED)
# Workaround for header-only Boost issue in older versions of cmake:
# https://gitlab.kitware.com/cmake/cmake/-/issues/18643
add_library(${PROJECT_NAME}_boost INTERFACE IMPORTED)
set_property(TARGET ${PROJECT_NAME}_boost PROPERTY
  INTERFACE_INCLUDE_DIRECTORIES ${Boost_INCLUDE_DIR})

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to pkg-config
  find_package(PkgConfig)
  pkg_check_modules(EIGEN3 REQUIRED eigen3)
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()
add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
  DEPENDS Boost OpenCV EIGEN3
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    camera_calibration_parsers
    cv_bridge
    geometry_msgs
    image_geometry
    image_transport
    message_filters
    nav_msgs
    nodelet
    roscpp
    rospy
    std_msgs
    swri_math_util
    swri_opencv_util
    swri_roscpp
    tf
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/draw_util.cpp
  src/geometry_util.cpp
  src/image_matching.cpp
  src/image_normalization.cpp
  src/image_warp_util.cpp
  src/replace_colors.cpp
  src/rolling_normalization.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
set_property(TARGET ${PROJECT_NAME}
  PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PROJECT_NAME}_boost
)

add_library(${PROJECT_NAME}_nodes SHARED
  src/nodes/blend_images_node.cpp
  src/nodes/crosshairs_node.cpp
  src/nodes/contrast_stretch_node.cpp
  src/nodes/draw_polygon_node.cpp
  src/nodes/draw_text_node.cpp
  src/nodes/dummy_image_publisher_node.cpp
  src/nodes/image_pub_node.cpp
  src/nodes/normalization_image_node.cpp
  src/nodes/normalize_response_node.cpp
  #src/nodes/replace_colors_node.cpp
  src/nodes/rotate_image_node.cpp
  src/nodes/scale_image_node.cpp
  src/nodes/warp_image_node.cpp
)
target_compile_definitions(${PROJECT_NAME}_nodes
  PRIVATE "COMPOSITION_BUILDING_DLL")
rclcpp_components_register_nodes(${PROJECT_NAME}_nodes "swri_image_util::BlendImagesNode")
rclcpp_components_register_nodes(${PROJECT_NAME}_nodes "swri_image_util::ContrastStretchNode")
rclcpp_components_register_nodes(${PROJECT_NAME}_nodes "swri_image_util::CrosshairsNode")
rclcpp_components_register_nodes(${PROJECT_NAME}_nodes "swri_image_util::DrawPolygonNode")
rclcpp_components_register_nodes(${PROJECT_NAME}_nodes "swri_image_util::DrawTextNode")
rclcpp_components_register_nodes(${PROJECT_NAME}_nodes "swri_image_util::DummyImagePublisherNode")
rclcpp_components_register_nodes(${PROJECT_NAME}_nodes "swri_image_util::ImagePubNode")
rclcpp_components_register_nodes(${PROJECT_NAME}_nodes "swri_image_util::NormalizationImageNode")
rclcpp_components_register_nodes(${PROJECT_NAME}_nodes "swri_image_util::NormalizeResponseNode")
# rclcpp_components_register_nodes(${PROJECT_NAME}_nodes "swri_image_util::ReplaceColorsNode")
rclcpp_components_register_nodes(${PROJECT_NAME}_nodes "swri_image_util::RotateImageNode")
rclcpp_components_register_nodes(${PROJECT_NAME}_nodes "swri_image_util::ScaleImageNode")
rclcpp_components_register_nodes(${PROJECT_NAME}_nodes "swri_image_util::WarpImageNode")
target_link_libraries(${PROJECT_NAME}_nodes ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_nodes
  ament_index_cpp
)

add_executable(blend_images src/nodes/blend_images_node.cpp)
target_link_libraries(blend_images ${catkin_LIBRARIES})

add_executable(contrast_stretch src/nodes/contrast_stretch.cpp)
target_link_libraries(contrast_stretch ${catkin_LIBRARIES})

add_executable(draw_text src/nodes/draw_text.cpp)
target_link_libraries(draw_text ${catkin_LIBRARIES})

add_executable(dummy_image_publisher src/nodes/dummy_image_publisher.cpp)
target_link_libraries(dummy_image_publisher ${catkin_LIBRARIES})

add_executable(normalization_image_generator_node src/nodes/normalization_image_generator_node.cpp)
target_link_libraries(normalization_image_generator_node ${PROJECT_NAME})

add_executable(normalize_response src/nodes/normalize_response.cpp)
target_link_libraries(normalize_response ${catkin_LIBRARIES})

add_executable(rotate_image src/nodes/rotate_image.cpp)
target_link_libraries(rotate_image ${catkin_LIBRARIES})

add_executable(scale_image src/nodes/scale_image.cpp)
target_link_libraries(scale_image ${catkin_LIBRARIES})

add_executable(replace_colors src/nodes/replace_colors.cpp)
target_link_libraries(replace_colors ${catkin_LIBRARIES})

swri_nodelet_add_node(draw_polygon ${PROJECT_NAME} DrawPolygonNodelet)
target_link_libraries(draw_polygon ${PROJECT_NAME}_nodelets ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

swri_nodelet_add_node(image_pub ${PROJECT_NAME} ImagePubNodelet)
target_link_libraries(image_pub ${PROJECT_NAME}_nodelets ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

swri_nodelet_add_node(warp_image ${PROJECT_NAME} WarpImageNodelet)
target_link_libraries(warp_image ${PROJECT_NAME}_nodelets ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

swri_nodelet_add_node(crosshairs ${PROJECT_NAME} CrosshairsNodelet)
target_link_libraries(crosshairs ${PROJECT_NAME}_nodelets ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_image_util test/image_util.test test/test_image_util.cpp)
  target_link_libraries(test_image_util ${PROJECT_NAME})
endif()

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
    ${PROJECT_NAME}_nodes
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_export_dependencies(ament_cmake)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_nodes)

ament_package()
