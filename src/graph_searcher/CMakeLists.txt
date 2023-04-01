cmake_minimum_required(VERSION 2.8.3)
project(graph_searcher)

find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    std_msgs
    occ_grid
    nav_msgs
    visualization_msgs 
    visualization
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
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

add_executable( graph_search_node 
    # src/graph_search_node.cpp
    src/graph_based_node.cpp
    src/a_star.cpp
    #src/JPS_utils.cpp
    #src/JPS_searcher.cpp
    )

target_link_libraries(graph_search_node 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} 
)

add_executable ( pcd_save 
    src/pcd/pcd_save.cpp )

target_link_libraries( pcd_save
${catkin_LIBRARIES}
${PCL_LIBRARIES} )

add_executable ( pcd_read 
    src/pcd/pcd_read.cpp )

target_link_libraries( pcd_read
${catkin_LIBRARIES}
${PCL_LIBRARIES} )