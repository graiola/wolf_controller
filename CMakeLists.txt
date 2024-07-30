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
find_package(catkin REQUIRED tf tf_conversions controller_interface control_toolbox wolf_hardware_interface
    realtime_tools ddynamic_reconfigure rviz_visual_tools rt_logger wolf_controller_core)
# RT gui
find_package(rt_gui QUIET)
# OCS2
find_package(ocs2_msgs QUIET)
# ADVR Packages
find_package(OpenSoT REQUIRED)
find_package(XBotInterface REQUIRED)
find_package(ModelInterfaceRBDL REQUIRED)

if(${rt_gui_FOUND})
    message(STATUS "Found rt_gui, creating interface for wolf_controller.")
    add_definitions(-DRT_GUI)
else()
    message(STATUS "rt_gui not found.")
endif()

if(${ocs2_msgs_FOUND})
    message(STATUS "Found ocs2_msgs, creating OCS2 interface for wolf_controller.")
    add_definitions(-DOCS2)
else()
    message(STATUS "ocs2_msgs not found.")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS wolf_controller_core
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${rt_gui_INCLUDE_DIRS}
    ${ocs2_msgs_INCLUDE_DIRS}
    ${OpenSoT_INCLUDE_DIRS}
    ${XBotInterface_INCLUDE_DIRS}
    ${ModelInterfaceRBDL_INCLUDE_DIRS}
)

add_library(wolf_controller_ros src/controller_wrapper.cpp src/controller_plugin.cpp)
target_link_libraries(wolf_controller_ros ${catkin_LIBRARIES} ${rt_gui_LIBRARIES})

add_executable(plot_node src/plot_node.cpp)
target_link_libraries(plot_node ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS wolf_controller_ros plot_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES controller_plugin.xml
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})  

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY params/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY resources/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resources
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  PATTERN ".svn" EXCLUDE)
  
install(DIRECTORY plotjuggler/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/plotjuggler
  PATTERN ".svn" EXCLUDE)

catkin_install_python(PROGRAMS scripts/keyboard_node
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
