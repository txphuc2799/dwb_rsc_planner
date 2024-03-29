cmake_minimum_required(VERSION 3.0.2)
project(dwb_rsc_plugins)

find_package(catkin REQUIRED COMPONENTS 
  angles 
  dwb_rsc_local_planner 
  dynamic_reconfigure 
  nav_2d_msgs 
  nav_2d_utils 
  nav_core2 
  pluginlib 
  roscpp
)

generate_dynamic_reconfigure_options(cfg/KinematicParams.cfg)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS angles dwb_rsc_local_planner dynamic_reconfigure nav_2d_msgs nav_2d_utils nav_core2 pluginlib roscpp
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_library(rsc_simple_goal_checker src/simple_goal_checker.cpp)
target_link_libraries(rsc_simple_goal_checker ${catkin_LIBRARIES})
add_dependencies(rsc_simple_goal_checker ${catkin_EXPORTED_TARGETS} ${dwb_rsc_plugins_EXPORTED_TARGETS})

add_library(rsc_stopped_goal_checker src/stopped_goal_checker.cpp)
target_link_libraries(rsc_stopped_goal_checker rsc_simple_goal_checker ${catkin_LIBRARIES})
add_dependencies(rsc_stopped_goal_checker ${catkin_EXPORTED_TARGETS} ${dwb_rsc_plugins_EXPORTED_TARGETS})

add_library(rsc_standard_traj_generator
            src/standard_traj_generator.cpp
            src/limited_accel_generator.cpp
            src/kinematic_parameters.cpp
            src/xy_theta_iterator.cpp)
target_link_libraries(rsc_standard_traj_generator ${catkin_LIBRARIES})
add_dependencies(rsc_standard_traj_generator ${catkin_EXPORTED_TARGETS} ${dwb_rsc_plugins_EXPORTED_TARGETS})

if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(vtest test/velocity_iterator_test.cpp)

  find_package(rostest REQUIRED)
  add_rostest_gtest(goal_checker test/goal_checker.launch test/goal_checker.cpp)
  target_link_libraries(goal_checker simple_goal_checker stopped_goal_checker ${GTEST_LIBRARIES})

  add_rostest_gtest(twist_gen_test test/twist_gen.launch test/twist_gen.cpp)
  target_link_libraries(twist_gen_test standard_traj_generator ${GTEST_LIBRARIES})

  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()
endif (CATKIN_ENABLE_TESTING)

install(TARGETS rsc_simple_goal_checker rsc_stopped_goal_checker rsc_standard_traj_generator
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES dwb_rsc_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
