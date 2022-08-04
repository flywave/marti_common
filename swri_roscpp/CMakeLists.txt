cmake_minimum_required(VERSION 3.0.2)

project(swri_roscpp)

set(BUILD_DEPS
  diagnostic_updater
  dynamic_reconfigure 
  marti_common_msgs
  marti_introspection_msgs
  nav_msgs
  roscpp
  std_msgs
  std_srvs
)

set(RUNTIME_DEPS
  diagnostic_updater 
  dynamic_reconfigure
  marti_common_msgs
  marti_introspection_msgs
  nav_msgs 
  roscpp
  std_msgs
  std_srvs
)

find_package(Boost REQUIRED COMPONENTS thread)

find_package(PkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)

### Catkin ###
find_package(catkin REQUIRED COMPONENTS ${BUILD_DEPS})
include_directories(include)
include_directories(SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${YamlCpp_INCLUDE_DIR}
)

if (CATKIN_ENABLE_TESTING)
  find_package(message_generation REQUIRED)
  find_package(message_runtime REQUIRED)
  add_message_files(DIRECTORY msg FILES
    TestTopicServiceRequest.msg
    TestTopicServiceResponse.msg
    )
  generate_messages(DEPENDENCIES marti_common_msgs)
endif()

catkin_package(CATKIN_DEPENDS ${RUNTIME_DEPS}
  DEPENDS Boost
  INCLUDE_DIRS include
  LIBRARIES ${YamlCpp_LIBRARIES}
  CFG_EXTRAS swri_roscpp-extras.cmake)

### Build Test Node ###
add_executable(dynamic_parameters_test_node src/nodes/dynamic_parameters_test_node.cpp)
target_link_libraries(dynamic_parameters_test_node
  ${catkin_LIBRARIES}
  ${YamlCpp_LIBRARIES}
  Boost::thread
)
set_target_properties(dynamic_parameters_test_node PROPERTIES COMPILE_FLAGS "-std=c++11")

add_executable(subscriber_test src/nodes/subscriber_test.cpp)
target_include_directories(subscriber_test
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(subscriber_test
  diagnostic_updater
  marti_common_msgs
  nav_msgs
  rclcpp
  std_msgs
  std_srvs
)

add_executable(storing_subscriber_test src/nodes/storing_subscriber_test.cpp)
target_include_directories(storing_subscriber_test
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(storing_subscriber_test
  diagnostic_updater
  marti_common_msgs
  nav_msgs
  rclcpp
  std_msgs
  std_srvs
  )

add_executable(service_server_test src/nodes/service_server_test.cpp)
target_link_libraries(service_server_test
  ${catkin_LIBRARIES}
  Boost::thread
)

add_executable(timer_test src/nodes/timer_test.cpp)
target_include_directories(timer_test
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(timer_test
  diagnostic_updater
  marti_common_msgs
  nav_msgs
  rclcpp
  std_msgs
  std_srvs
  )

if (BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Boost REQUIRED COMPONENTS system)

  rosidl_generate_interfaces(${PROJECT_NAME}
    msg/TestTopicServiceRequest.msg
    msg/TestTopicServiceResponse.msg
    DEPENDENCIES std_msgs marti_common_msgs
  )

  add_rostest_gtest(test_dynamic_parameters
    test/test_dynamic_parameters.test
    test/test_dynamic_parameters.cpp)
  target_link_libraries(test_dynamic_parameters ${catkin_LIBRARIES} ${YamlCpp_LIBRARIES})
  set_target_properties(test_dynamic_parameters PROPERTIES COMPILE_FLAGS "-std=c++11")

  ### Test the TopicService server and client ###
  # This is a little different from the way tests are normally declared because we
  # have a single binary that contains both server and client tests, but the tests
  # themselves are run in separate .test files.
  add_executable(test_topic_service
    test/topic_service_test.cpp)
  target_link_libraries(test_topic_service
    ${catkin_LIBRARIES}
    ${GTEST_LIBRARIES}
    Boost::thread
  )
  add_dependencies(test_topic_service ${${PROJECT_NAME}_EXPORTED_TARGETS})
  add_rostest(test/topic_service_client_test.test DEPENDENCIES test_topic_service)
  add_rostest(test/topic_service_server_test.test DEPENDENCIES test_topic_service)
endif()

### Install Test Node and Headers ###
install(TARGETS subscriber_test
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(PROGRAMS scripts/service_splitter.py
  DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch/
  DESTINATION launch
)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(diagnostic_updater)
ament_export_dependencies(marti_common_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(std_srvs)
ament_export_include_directories(include)

ament_package(CONFIG_EXTRAS cmake/swri_roscpp-extras.cmake )
