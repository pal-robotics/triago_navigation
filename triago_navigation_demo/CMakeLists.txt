cmake_minimum_required(VERSION 3.8)
project(triago_navigation_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(ncl_utils REQUIRED)
find_package(rclpy REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(pal_nav2_planners REQUIRED)
find_package(pal_module_cmake REQUIRED)

ament_python_install_package(${PROJECT_NAME})

set(dependencies
  nav2_msgs
  nav_msgs
  ncl_utils
  rclpy
  rcl_interfaces
  pal_nav2_planners
)

install(DIRECTORY module DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/pal_circular_path_following_demo.py
  ${PROJECT_NAME}/pal_cartesian_navigation_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

if(pal_module_cmake_FOUND)
  pal_register_modules(module/circular_path_following_demo.yaml
                        module/cartesian_navigation_demo.yaml
                        module/velocity_controller_demo.yaml)
elseif(DEFINED ENV{PAL_DISTRO})
  message(FATAL_ERROR "The pal_module_cmake package was not found. Modules not registered.")
endif()
                      

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  nav_msgs
  rclcpy
  nav2_msgs
  ncl_utils
)

ament_package()
