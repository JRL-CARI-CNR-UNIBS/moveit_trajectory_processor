cmake_minimum_required(VERSION 3.5)
project(moveit_trajectory_processor)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  if(CMAKE_BUILD_TYPE MATCHES "Release")
    add_compile_options(-funroll-loops -Wall -Ofast -flto)
  endif()
  if(CMAKE_BUILD_TYPE MATCHES "Debug")
    add_compile_options(-Wall -g -Og)
  endif()
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

find_package(trajectories_processors_lib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

set(DEPS_LIB
    trajectories_processors_lib
    )
set(DEPS_TARGET
    trajectories_processors_lib::trajectories_processors_lib
    )
set(ROS_DEPS
    rclcpp
    moveit_ros_planning_interface
    )

# Declare a C++ library
add_library(${PROJECT_NAME} SHARED
  src/moveit_trajectory_processor.cpp
)
target_link_libraries(${PROJECT_NAME} PUBLIC ${DEPS_TARGET})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${ROS_DEPS})

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPS_LIB} ${ROS_DEPS})

# Install
install(DIRECTORY include/${PROJECT_NAME}
    DESTINATION include)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(EXPORT ${PROJECT_NAME}Targets
    DESTINATION "share/${PROJECT_NAME}/cmake"
    NAMESPACE ${PROJECT_NAME}::
    FILE ${PROJECT_NAME}Targets.cmake
)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    VERSION 0.0.1
    COMPATIBILITY AnyNewerVersion)
configure_package_config_file(
    ${CMAKE_SOURCE_DIR}/cmake_config/${PROJECT_NAME}Config.cmake.in ${PROJECT_NAME}Config.cmake
    INSTALL_DESTINATION share/${PROJECT_NAME}/cmake
    PATH_VARS CMAKE_INSTALL_INCLUDEDIR CMAKE_INSTALL_LIBDIR
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    DESTINATION "share/${PROJECT_NAME}/cmake")

# make uninstall
if(ADD_CUSTOM_UNINSTALL)
    message(STATUS "Adding custom uninstall")
    add_custom_target("uninstall" COMMENT "Uninstall installed files")
    add_custom_command(
        TARGET "uninstall"
        POST_BUILD
        COMMENT "Uninstall files with install_manifest.txt"
        COMMAND xargs rm -vf < install_manifest.txt || echo Nothing in
        install_manifest.txt to be uninstalled!
    )
endif()

# if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
#  # the following line skips the linter which checks for copyrights
#  # comment the line when a copyright and license is added to all source files
#  set(ament_cmake_copyright_FOUND TRUE)
#  # the following line skips cpplint (only works in a git repo)
#  # comment the line when this package is in a git repo and when
#  # a copyright and license is added to all source files
#  set(ament_cmake_cpplint_FOUND TRUE)
#  ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
