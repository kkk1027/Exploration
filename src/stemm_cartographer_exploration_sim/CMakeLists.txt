cmake_minimum_required(VERSION 3.19)
project(stemm_cartographer_exploration_sim)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/session_driver scripts/patch_sim_time scripts/frontier_arbiter
        scripts/frontier_merge
        scripts/collision_probe scripts/escape_supervisor scripts/wait_robot_ready
        scripts/vision_obstacle_grid scripts/explore_watchdog scripts/nav2_lifecycle_nudge
        scripts/nav_plan_audit scripts/fake_base
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY scripts/sim_shims DESTINATION lib/${PROJECT_NAME})
ament_package()
