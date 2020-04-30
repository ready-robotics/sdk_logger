cmake_minimum_required(VERSION 3.5.2)
project(sdk_logger_user)

if(NOT CMAKE_BUILD_TYPE STREQUAL Debug)
  set(CMAKE_BUILD_TYPE Release)
endif()
 
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Werror -Wextra")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED sdk_logger)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sdk_logger_user
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_user_node src/user_node.cpp)
add_dependencies(${PROJECT_NAME}_user_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_user_node
  ${catkin_LIBRARIES}
)
