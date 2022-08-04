cmake_minimum_required(VERSION 3.0.2)
project(swri_opencv_util)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(swri_math_util REQUIRED)

find_package(OpenCV REQUIRED core imgproc highgui)

find_package(Boost REQUIRED COMPONENTS serialization thread) 

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS cv_bridge swri_math_util
  DEPENDS Boost
)

include_directories(include ${catkin_INCLUDE_DIRS})
  
add_library(${PROJECT_NAME} SHARED
  src/blend.cpp
  src/models.cpp
  src/model_fit.cpp
  src/show.cpp
  src/convert.cpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  Boost::serialization
  Boost::thread
)
target_compile_definitions(${PROJECT_NAME} PUBLIC "-DACCEPT_USE_OF_DEPRECATED_PROJ_API_H")

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(cv_bridge)
ament_export_dependencies(swri_math_util)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
