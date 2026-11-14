cmake_minimum_required(VERSION 3.19)
project(stemm_cartographer_exploration_sim)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/session_driver scripts/patch_sim_time scripts/frontier_arbiter
        scripts/frontier_merge
        scripts/urdf_publisher
        scripts/collision_probe scripts/escape_supervisor scripts/wait_robot_ready
        scripts/vision_obstacle_grid scripts/explore_watchdog scripts/nav2_lifecycle_nudge
        scripts/nav_plan_audit scripts/fake_base
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY scripts/sim_shims DESTINATION lib/${PROJECT_NAME})
# S5 M1：车体画像包 + 构型 profile（几何唯一事实源）
install(DIRECTORY scripts/robot_model DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config/robot_profiles DESTINATION share/${PROJECT_NAME}/config)
# S5 M2：扫掠可通行性内核（构型无关）
install(DIRECTORY scripts/sweep_kernel DESTINATION lib/${PROJECT_NAME})
ament_package()
