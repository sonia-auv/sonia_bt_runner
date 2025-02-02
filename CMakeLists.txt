cmake_minimum_required(VERSION 3.8)
project(sonia_bt_runner)

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(sonia_common_ros2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include)

file(GLOB SOURCES src/*.cpp)

add_executable(${PROJECT_NAME} ${SOURCES})

install(
    DIRECTORY include/
    DESTINATION include
)

install(TARGETS
${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME})

ament_target_dependencies(${PROJECT_NAME} rclcpp sonia_common_ros2 std_msgs std_srvs behaviortree_cpp) 

ament_package()