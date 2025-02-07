cmake_minimum_required(VERSION 3.8)
project(sonia_bt_runner)

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#==============================================================================
#C M A K E   C O N F I G U R A T I O N
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(sonia_common_ros2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

#==============================================================================
#I N C L U D E   F I L E S

include_directories(include)

#file(GLOB SOURCES src/*.cpp)

add_executable(TARGET_NAME_GEN src/gen_nodes.cpp)
add_executable(TARGET_NAME_MAIN src/main.cpp)

install(
    DIRECTORY include/
    DESTINATION include
)

install(TARGETS
    ${TARGET_NAME_MAIN}
    ${TARGET_NAME_GEN}
    DESTINATION lib/${PROJECT_NAME})

ament_target_dependencies(${TARGET_NAME_MAIN} rclcpp sonia_common_ros2 std_msgs std_srvs behaviortree_cpp)
ament_target_dependencies(${TARGET_NAME_GEN} rclcpp sonia_common_ros2 std_msgs std_srvs behaviortree_cpp) 

ament_package()