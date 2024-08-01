# \file     CMakeLists.txt
# \author   Olivier Lavoie
# \date     10/16/2015
# \copyright    2017 Club SONIA AUV, ETS. All rights reserved.
# Use of this source code is governed by the MIT license that can be
# found in the LICENSE file.

#===============================================================================
# G L O B A L   P R O J E C T   C O N F I G U R A T I O N
cmake_minimum_required(VERSION 3.5)
project(sonia_bt_runner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING
            "Choose the type of build: None Debug Release RelWithDebInfo MinSizeRel."
            FORCE)
endif (NOT CMAKE_BUILD_TYPE)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++11 -O3")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

#===============================================================================
# C A T K I N   C O N F I G U R A T I O N

find_package( catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        behaviortree_cpp
        sonia_common
        )
# Declare this project as a catkin package and export the necessary build flags
catkin_package(
        INCLUDE_DIRS src
        LIBRARIES
        CATKIN_DEPENDS
        roscpp
        std_msgs
        behaviortree_cpp
        sonia_common
)



#===============================================================================
# I N C L U D E   F I L E S

set(sonia_bt_runner_INCLUDE_DIRS "include")
set(sonia_bt_runner_SRC_DIR "src")

file(GLOB_RECURSE sonia_bt_runner_FILES
        "${sonia_bt_runner_SRC_DIR}/*.cpp"
        "${sonia_bt_runner_SRC_DIR}/*.hpp")

list(REMOVE_ITEM sonia_bt_runner_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/${sonia_bt_runner_SRC_DIR}/main.cpp)
list(REMOVE_ITEM sonia_bt_runner_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/${sonia_bt_runner_SRC_DIR}/gen_nodes.cpp)

# Include all the directories where source can be found.
include_directories(
        ${sonia_bt_runner_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)
#==========================================================================
# C R E A T E   E X E C U T A B L E
set( TARGET_NAME_MAIN run_mission )
add_executable( ${TARGET_NAME_MAIN}
        ${sonia_bt_runner_SRC_DIR}/main.cpp ${sonia_bt_runner_FILES}
)
target_link_libraries( ${TARGET_NAME_MAIN} ${catkin_LIBRARIES})
set_target_properties( ${TARGET_NAME_MAIN} PROPERTIES LINKER_LANGUAGE CXX )
add_dependencies(${TARGET_NAME_MAIN} ${catkin_EXPORTED_TARGETS})

set( TARGET_NAME_GEN gen_nodes )
add_executable( ${TARGET_NAME_GEN}
        ${sonia_bt_runner_SRC_DIR}/gen_nodes.cpp ${sonia_bt_runner_FILES}
)
target_link_libraries( ${TARGET_NAME_GEN} ${catkin_LIBRARIES})
set_target_properties( ${TARGET_NAME_GEN} PROPERTIES LINKER_LANGUAGE CXX )
add_dependencies(${TARGET_NAME_GEN} ${catkin_EXPORTED_TARGETS})
#==========================================================================
# U N I T   T E S T S