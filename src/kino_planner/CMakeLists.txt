cmake_minimum_required(VERSION 2.8.3)
project(kino_planner)

find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    std_msgs
    nav_msgs
    visualization_msgs 
    occ_grid
    visualization
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES graph_searcher
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

add_executable( kino_plan_node 
    src/kino_based_node.cpp
    src/kino_a_star.cpp
    )

target_link_libraries(kino_plan_node 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} 
)