cmake_minimum_required(VERSION 3.0.2)
project(dwb_rsc_local_planner)


find_package(catkin REQUIRED COMPONENTS
  dwb_rsc_msgs
  geometry_msgs
  nav_2d_msgs
  nav_2d_utils
  nav_core2
  nav_msgs
  pluginlib
  roscpp
  sensor_msgs
  tf
  visualization_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        dwb_rsc_msgs geometry_msgs nav_2d_msgs nav_2d_utils nav_core2
        nav_msgs pluginlib roscpp sensor_msgs tf visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/dwb_rsc_local_planner.cpp
                            src/backwards_compatibility.cpp
                            src/publisher.cpp
                            src/illegal_trajectory_tracker.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_library(debug_${PROJECT_NAME} src/debug_dwb_rsc_local_planner.cpp)
target_link_libraries(debug_${PROJECT_NAME} ${catkin_LIBRARIES} ${PROJECT_NAME})
add_dependencies(debug_${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_library(rsc_trajectory_utils src/trajectory_utils.cpp)
target_link_libraries(rsc_trajectory_utils ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/planner_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} debug_${PROJECT_NAME})
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME planner_node PREFIX "")

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()

  catkin_add_gtest(utils_test test/utils_test.cpp)
  target_link_libraries(utils_test trajectory_utils)
endif()

install(TARGETS ${PROJECT_NAME}_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS ${PROJECT_NAME} debug_${PROJECT_NAME} rsc_trajectory_utils
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES dwb_rsc_local_planner.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        FILES_MATCHING PATTERN "*.launch"
)
