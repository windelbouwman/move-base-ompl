cmake_minimum_required(VERSION 2.8.3)
project(move_base_ompl)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    costmap_2d
    geometry_msgs
    nav_core
    navfn
    nav_msgs
    pluginlib
    roscpp
    tf
    base_local_planner
)

find_package(OMPL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ompl_global_planner_lib
  CATKIN_DEPENDS
    costmap_2d
    geometry_msgs
    nav_core
    navfn
    nav_msgs
    pluginlib
    roscpp
    tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(ompl_global_planner_lib
  src/ompl_global_planner.cpp
)
target_link_libraries(ompl_global_planner_lib ${catkin_LIBRARIES} ${OMPL_LIBRARIES})

add_dependencies(ompl_global_planner_lib ${PROJECT_NAME}_gencfg)

install(TARGETS ompl_global_planner_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES ompl_gp_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
