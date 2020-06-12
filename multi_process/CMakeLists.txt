cmake_minimum_required(VERSION 3.5.1)
project(log_process)

if(NOT CMAKE_BUILD_TYPE STREQUAL Debug) 							
	set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Werror -Wextra -pthread -g")
set(CMAKE_CXX_STANDARD 11)											
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} ${CMAKE_CXX_FLAGS} -Weffc++ -std=c++11")	
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} ${CMAKE_CXX_FLAGS} -std=c++11")	


set(LOG_PROCESS_EXE log_process)
set(USER_PROCESS_EXE user_process)

include_directories(
    ../spdlog/include
)

add_executable(
	${LOG_PROCESS_EXE}
	log_process.cpp
)

add_executable(
	${PROCESS_ONE_EXE}
	user_process.cpp
)



