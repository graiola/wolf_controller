cmake_minimum_required(VERSION 3.5)
project(wolf_controller_ros)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++17" COMPILER_SUPPORTS_CXX17)
check_cxx_compiler_flag("-std=c++14" COMPILER_SUPPORTS_CXX14)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wmaybe-uninitialized -Wuninitialized")
elseif(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wmaybe-uninitialized -Wuninitialized")
elseif(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wmaybe-uninitialized -Wuninitialized")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wmaybe-uninitialized -Wuninitialized")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 nor C++14 support. Please use a different C++ compiler.")
endif()

# ROS
find_package(catkin QUIET)
find_package(ament_cmake QUIET)

if(${catkin_FOUND})
    set(ROS_VERSION "ros")
    catkin_package(INCLUDE_DIRS include LIBRARIES wolf_controller_ros)
    add_definitions(-DROS)
    message(STATUS "Add ROS definition")
elseif(${ament_cmake_FOUND})
    set(ROS_VERSION "ros2")
    ament_export_include_directories(include)
    ament_export_libraries(wolf_controller_ros)
    ament_package()
    add_definitions(-DROS2)
    message(STATUS "Add ROS2 definition")
endif()

# ROS pkgs
find_package(tf2 REQUIRED)
find_package(controller_interface REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(wolf_controller_utils REQUIRED)
find_package(wolf_hardware_interface REQUIRED)
find_package(wolf_controller_core REQUIRED)
find_package(wolf_wbid REQUIRED)
find_package(wolf_msgs REQUIRED)
find_package(wolf_estimation REQUIRED)

# RT logger
find_package(rt_logger QUIET)
# OCS2
find_package(ocs2_msgs QUIET)
# DDYNAMIC RECONFIGURE
find_package(ddynamic_reconfigure QUIET)

if(${rt_logger_FOUND})
    message(STATUS "Found rt_logger, creating logger for wolf_controller_ros.")
    add_definitions(-DRT_LOGGER)
else()
    message(STATUS "rt_logger not found.")
endif()

if(${ocs2_msgs_FOUND})
    message(STATUS "Found ocs2_msgs, creating OCS2 interface for wolf_controller_ros.")
    add_definitions(-DOCS2)
else()
    message(STATUS "ocs2_msgs not found.")
endif()

if (${ddynamic_reconfigure_FOUND})
    message(STATUS "Found ddynamic_reconfigure, creating ddynamic reconfigure interface for wolf_controller_ros.")
    add_definitions(-DDDYNAMIC_RECONFIGURE)
else()
    message(STATUS "ddynamic_reconfigure not found.")
endif()

include_directories(
    include/${ROS_VERSION}
    ${tf2_INCLUDE_DIRS}
    ${controller_interface_INCLUDE_DIRS}
    ${wolf_controller_utils_INCLUDE_DIRS}
    ${wolf_hardware_interface_INCLUDE_DIRS}
    ${wolf_controller_core_INCLUDE_DIRS}
    ${wolf_wbid_INCLUDE_DIRS}
    ${wolf_estimation_INCLUDE_DIRS}
    ${wolf_msgs_INCLUDE_DIRS}
    ${realtime_tools_INCLUDE_DIRS}
    ${rviz_visual_tools_INCLUDE_DIRS}
    ${rt_logger_INCLUDE_DIRS}
    ${ddynamic_reconfigure_INCLUDE_DIRS}
    ${ocs2_msgs_INCLUDE_DIRS}
)

add_library(wolf_controller_ros src/${ROS_VERSION}/controller_wrapper.cpp src/${ROS_VERSION}/controller_plugin.cpp)
target_link_libraries(wolf_controller_ros
    ${tf2_LIBRARIES}
    ${controller_interface_LIBRARIES}
    ${wolf_controller_utils_LIBRARIES}
    ${wolf_hardware_interface_LIBRARIES}
    ${wolf_controller_core_LIBRARIES}
    ${wolf_wbid_LIBRARIES}
    ${wolf_estimation_LIBRARIES}
    ${realtime_tools_LIBRARIES}
    ${rviz_visual_tools_LIBRARIES}
    ${rt_logger_LIBRARIES}
    ${ddynamic_reconfigure_LIBRARIES}
    ${ocs2_msgs_LIBRARIES})

#add_executable(plot_node src/${ROS_VERSION}/plot_node.cpp)
#target_link_libraries(plot_node ${rviz_visual_tools_LIBRARIES})

install(
  DIRECTORY include/${ROS_VERSION}/${PROJECT_NAME}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include
)

install(TARGETS wolf_controller_ros #plot_node
    ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
)

install(FILES controller_plugin.xml
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/launch)

install(DIRECTORY params/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/params)

install(DIRECTORY resources/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/resources)

install(DIRECTORY rviz/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/rviz)

install(DIRECTORY plotjuggler/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/plotjuggler)

install(PROGRAMS scripts/keyboard_node
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME})

