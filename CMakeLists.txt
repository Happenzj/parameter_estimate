cmake_minimum_required(VERSION 2.8.3)
project(parameter_estimate)

set(CMAKE_CXX_STANDARD 11)
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3 -pthread" )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

find_package( Ceres REQUIRED )
find_package(catkin REQUIRED COMPONENTS 
             roscpp roboteq_diff_msgs std_msgs 
             nav_msgs sensor_msgs  tf )

find_package(Boost REQUIRED COMPONENTS system thread)

catkin_package(
   CATKIN_DEPENDS roscpp roboteq_diff_msgs std_msgs nav_msgs sensor_msgs tf 
)
set( THIRD_PARTY_LIBS
        ${catkin_LIBRARIES}
        ${CERES_LIBRARIES}
        -lboost_system
        )

include_directories(include ${catkin_INCLUDE_DIRS})

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

include_directories( ${CERES_INCLUDE_DIRS})

add_executable( parameter_estimate src/parameter_estimate.cpp)
target_link_libraries( parameter_estimate ${THIRD_PARTY_LIBS})