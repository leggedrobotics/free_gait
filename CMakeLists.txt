# Project configuration
cmake_minimum_required (VERSION 2.8)

message("      Including task free gait")

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})

# CMake modules
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

add_definitions(--std=c++11)

get_target_property(target_location_loco loco LOCATION)
if (target_location_loco STREQUAL "target_location_loco-NOTFOUND")
    add_subdirectory(../locomotionControl locomotionControl)
endif()

set(TASK_INCL ${TASK_INCL}
	${CMAKE_CURRENT_SOURCE_DIR}/include 
)


################
### INCLUDES ###
################
set(TASK_INCL ${TASK_INCL}

PARENT_SCOPE)

###############
### SOURCES ###
###############
set(TASK_SRCS ${TASK_SRCS}
	${CMAKE_CURRENT_SOURCE_DIR}/src/FreeGait_Task.cpp 
PARENT_SCOPE)

#################
### LIBRARIES ###
#################
set(TASK_LIBS ${TASK_LIBS} 
PARENT_SCOPE)
 
