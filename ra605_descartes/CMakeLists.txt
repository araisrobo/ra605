cmake_minimum_required(VERSION 2.8.3)
project(ra605_descartes)

find_package(catkin REQUIRED
  descartes_core
  descartes_moveit
  descartes_trajectory
  descartes_planner
  trajectory_msgs
  pluginlib
  roscpp 
  tf_conversions
  moveit_core 
  moveit_msgs
  moveit_ros_planning_interface
)
find_package(rosconsole_bridge REQUIRED)
find_package(LAPACK REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    moveit_core
    descartes_moveit
    pluginlib
    DEPENDS
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_tutorial1 src/tutorial1.cpp)
target_link_libraries(${PROJECT_NAME}_tutorial1 ${catkin_LIBRARIES})
install(TARGETS ${PROJECT_NAME}_tutorial1
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
set_target_properties(${PROJECT_NAME}_tutorial1
  PROPERTIES OUTPUT_NAME tutorial1
  PREFIX ""
)

add_executable(${PROJECT_NAME}_line src/line.cpp)
target_link_libraries(${PROJECT_NAME}_line ${catkin_LIBRARIES})
install(TARGETS ${PROJECT_NAME}_line
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
set_target_properties(${PROJECT_NAME}_line
  PROPERTIES OUTPUT_NAME line
  PREFIX ""
)

# TODO:
# add_library(${PROJECT_NAME}
#   src/ra605_robot_model.cpp
#   src/plugin_init.cpp
# )
# target_link_libraries(${PROJECT_NAME}
#   ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${Boost_LIBRARIES} ${LAPACK_LIBRARIES})

#############
## Install ##
#############

# install(TARGETS ${PROJECT_NAME}_tutorial1
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
