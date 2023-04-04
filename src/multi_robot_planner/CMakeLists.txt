cmake_minimum_required(VERSION 2.8.3)
project(multi_robot_planner)

find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    std_msgs
    nav_msgs
    occ_grid
    visualization
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES multi_robot_planner
  CATKIN_DEPENDS occ_grid
)

include_directories(
    include 
    SYSTEM 
    third_party
    ${catkin_INCLUDE_DIRS} 
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall") # -Wextra -Werror

add_executable(${PROJECT_NAME} 
    src/mapf_based_node.cpp
)

target_link_libraries(${PROJECT_NAME} 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} 
)