cmake_minimum_required(VERSION 3.8)
project(triago_2dnav)

find_package(ament_cmake_auto REQUIRED)

# PAL Robotics stricter build flags, other developers should not worry about
# this
find_package(ament_cmake_pal QUIET)

ament_auto_find_build_dependencies()

if(pal_module_cmake_FOUND)
  pal_register_modules(module/40_localization.yaml
                      module/40_navigation.yaml
                      module/40_slam.yaml)
elseif(DEFINED ENV{PAL_DISTRO})
  message(FATAL_ERROR "The pal_module_cmake package was not found. Modules not registered.")
endif()
if(pal_configuration_cmake_FOUND)
  pal_register_configs(
    CONFIGS
      params/40_nav2_controller_triago.yaml
    RESOURCE_NAME
      nav2_controller
  )
elseif(DEFINED ENV{PAL_DISTRO})
  message(FATAL_ERROR "The pal_configuration_cmake package was not found. Configurations not registered.")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
