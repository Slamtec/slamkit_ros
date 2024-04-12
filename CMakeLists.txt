cmake_minimum_required(VERSION 2.8.3)
project(slamkit_ros)

set(SLAMKITSDK_PATH "./sdk/slamkit/")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

FILE(GLOB SLAMKITSDK_SRC 
  "${SLAMKITSDK_PATH}/src/arch/linux/*.cpp"
  "${SLAMKITSDK_PATH}/src/hal/*.cpp"
  "${SLAMKITSDK_PATH}/src/*.cpp"
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  sensor_msgs
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
include_directories(${LIBUSB_INCLUDE_DIRS})
link_directories(${LIBUSB_LIBRARY_DIRS})

include_directories(
  ${SLAMKITSDK_PATH}/include
  ${SLAMKITSDK_PATH}/src
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

add_executable(slamkitNode src/node.cpp ${SLAMKITSDK_SRC})
target_link_libraries(slamkitNode ${catkin_LIBRARIES} ${LIBUSB_LIBRARIES})

add_executable(slamkitNodeClient src/client.cpp)
target_link_libraries(slamkitNodeClient ${catkin_LIBRARIES} ${LIBUSB_LIBRARIES})

install(TARGETS slamkitNode slamkitNodeClient
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

