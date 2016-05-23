cmake_minimum_required(VERSION 2.8.3)
project(im920teleop)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  geometry_msgs
)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
	message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

link_directories(
)

add_executable(im920teleop src/im920teleop.cpp)

target_link_libraries(im920teleop
  ${catkin_LIBRARIES}
)

