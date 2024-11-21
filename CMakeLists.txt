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
 
 ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
 ament_export_dependencies(${DEPS_LIB} ${ROS_DEPS})

# Install
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

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
