cmake_minimum_required(VERSION 3.8)
project(sonia_bt_runner)

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#==============================================================================
#C M A K E   C O N F I G U R A T I O N
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(sonia_common_ros2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

#==============================================================================
#I N C L U D E   F I L E S

pkg_check_modules(tinyxml2 REQUIRED tinyxml2)

set(sonia_bt_runner_INCLUDE_DIRS "include")
set(sonia_bt_runner_SRC_DIR "src")

file(GLOB_RECURSE sonia_bt_runner_FILES
        "${sonia_bt_runner_SRC_DIR}/**.cpp"
        "${sonia_bt_runner_SRC_DIR}/*.hpp")

list(REMOVE_ITEM sonia_bt_runner_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/${sonia_bt_runner_SRC_DIR}/main.cpp)
list(REMOVE_ITEM sonia_bt_runner_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/${sonia_bt_runner_SRC_DIR}/gen_nodes.cpp)

include_directories(
        include
        ${sonia_bt_runner_INCLUDE_DIRS}
)

#==========================================================================
# C R E A T E   E X E C U T A B L E
set( TARGET_NAME_MAIN run_server )
add_executable( ${TARGET_NAME_MAIN}
        ${sonia_bt_runner_SRC_DIR}/main.cpp ${sonia_bt_runner_FILES}
)

set( TARGET_NAME_GEN gen_nodes )
add_executable( ${TARGET_NAME_GEN}
        ${sonia_bt_runner_SRC_DIR}/gen_nodes.cpp ${sonia_bt_runner_FILES}
)

install(
    DIRECTORY include/
    DESTINATION include
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
    ${TARGET_NAME_MAIN}
    ${TARGET_NAME_GEN}
    DESTINATION lib/${PROJECT_NAME})

ament_target_dependencies(${TARGET_NAME_MAIN} rclcpp rclcpp_action sonia_common_ros2 tinyxml2 std_msgs std_srvs behaviortree_cpp)
ament_target_dependencies(${TARGET_NAME_GEN} rclcpp rclcpp_action sonia_common_ros2 tinyxml2 std_msgs std_srvs behaviortree_cpp)

ament_package()
