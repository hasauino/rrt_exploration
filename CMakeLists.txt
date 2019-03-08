cmake_minimum_required(VERSION 2.8.3)
project(rrt_exploration)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  visualization_msgs
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  PointArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


catkin_package( 

CATKIN_DEPENDS message_runtime
)





include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(global_rrt_detector src/global_rrt_detector.cpp src/functions.cpp src/mtrand.cpp)
target_link_libraries(global_rrt_detector ${catkin_LIBRARIES})

add_executable(local_rrt_detector src/local_rrt_detector.cpp src/functions.cpp src/mtrand.cpp)
target_link_libraries(local_rrt_detector ${catkin_LIBRARIES})


install(TARGETS global_rrt_detector local_rrt_detector
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(
  PROGRAMS scripts/assigner.py
           scripts/filter.py
           scripts/frontier_opencv_detector.py
           scripts/getfrontier.py
           scripts/functions.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch scripts
       DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
