cmake_minimum_required(VERSION 3.10)
project(tron2_controllers)

set(CMAKE_CXX_STANDARD 17)
find_package(Eigen3 REQUIRED)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  robot_common
  robot_description
  controller_interface
  hardware_interface
  std_msgs
  geometry_msgs
  onnxruntime_sdk
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  roscpp
  robot_common
  robot_description
  controller_interface
  hardware_interface
  std_msgs
  geometry_msgs
  onnxruntime_sdk
)

set(ONNXRUNTIME_SDK_LIB_FILES libonnxruntime.so libonnxruntime.so.1.10.0)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

link_directories(${CATKIN_DEVEL_PREFIX}/lib)

add_library(${PROJECT_NAME} SHARED
  src/ControllerBase.cpp
  src/SolefootController.cpp
  src/WheelfootController.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${ONNXRUNTIME_SDK_LIB_FILES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/. DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES tron2_controllers_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
