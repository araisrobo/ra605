cmake_minimum_required(VERSION 2.8.3)

project(ra605_descartes)

find_package(
  catkin REQUIRED
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


## Descartes MoveIt lib
add_library(path_gen src/path_gen.cpp )
target_link_libraries(path_gen ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_tutorial1 src/tutorial1.cpp)
target_link_libraries(${PROJECT_NAME}_tutorial1 ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_line src/line.cpp)
target_link_libraries(${PROJECT_NAME}_line ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_cycleTime src/cycleTime.cpp)
target_link_libraries(${PROJECT_NAME}_cycleTime path_gen ${catkin_LIBRARIES})

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(GTest REQUIRED)
  include_directories(${GTEST_INCLUDE_DIRS})
  
  set(UTEST_SRC_FILES test/utest.cpp)

  add_rostest_gtest(${PROJECT_NAME}_utest test/launch/utest.launch ${UTEST_SRC_FILES})
  target_link_libraries(${PROJECT_NAME}_utest path_gen ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
endif()

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_tutorial1
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
set_target_properties(${PROJECT_NAME}_tutorial1
  PROPERTIES OUTPUT_NAME tutorial1
  PREFIX ""
)

install(TARGETS ${PROJECT_NAME}_line
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
set_target_properties(${PROJECT_NAME}_line
  PROPERTIES OUTPUT_NAME line
  PREFIX ""
)
  
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
