cmake_minimum_required(VERSION 3.5.1)
project(log_process)

if(NOT CMAKE_BUILD_TYPE STREQUAL Debug) 							
	set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Werror -Wextra")
set(CMAKE_CXX_STANDARD 11)											
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} ${CMAKE_CXX_FLAGS} -Weffc++ -std=c++11")	
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} ${CMAKE_CXX_FLAGS} -std=c++11")	

set(LOG_PROCESS_EXE log_process)
set(PROCESS_ONE_EXE proc_one)
set(PROCESS_TWO_EXE proc_two)

include_directories(
    ../spdlog/include
)

add_executable(
	${LOG_PROCESS_EXE}
	log_process.cpp
)

add_executable(
	${PROCESS_ONE_EXE}
	proc_one.cpp
)

add_executable(
	${PROCESS_TWO_EXE}
	proc_two.cpp
)



