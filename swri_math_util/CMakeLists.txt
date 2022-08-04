cmake_minimum_required(VERSION 3.0.2)
project(swri_math_util)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

find_package(Boost REQUIRED COMPONENTS random thread)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
  DEPENDS Boost
)

include_directories(include ${catkin_INCLUDE_DIRS})
  
add_library(${PROJECT_NAME}
  src/math_util.cpp
  src/trig_util.cpp
  src/random.cpp
  src/interpolation_1d.cpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Boost::random
  Boost::thread
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_trig_util test/test_trig_util.cpp)
  target_link_libraries(test_trig_util ${PROJECT_NAME})

  ament_add_gtest(test_math_util test/test_math_util.cpp)
  target_link_libraries(test_math_util ${PROJECT_NAME})

  ament_add_gtest(test_random test/test_random.cpp)
  target_link_libraries(test_random ${PROJECT_NAME})
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
ament_export_include_directories(include ${rclcpp_INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME})

ament_package()
