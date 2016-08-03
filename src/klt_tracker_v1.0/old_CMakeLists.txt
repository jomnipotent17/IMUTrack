# Project name is not mandatory, but you should use it
project(affineTracker_gpu)

# States that CMake required version must be >= 2.6
cmake_minimum_required(VERSION 2.6)

# Appends the cmake/modules path inside the MAKE_MODULE_PATH variable which stores the
# directories of additional CMake modules (eg MacroOutOfSourceBuild.cmake):
############################### OPENCV ################################
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR})
find_package( OpenCV )
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
LINK_DIRECTORIES(${OpenCV_LIBRARY_DIRS})

#message(STATUS "OpenCV_INCLUDE_DIRS ="  ${OpenCV_INCLUDE_DIRS})
#message(STATUS "OpenCV_LIBRARY_DIRS ="  ${OpenCV_LIBRARY_DIRS})
################################ IPP ################################
#set(IPP_FOUND)
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR})
find_package( IPP )

#    set(IPP_LIBS ${A}ippcore${B} ${A}ipps${B}  ${A}ippi${B})
#                 ${A}svml${B} ${A}imf${B} ${A}iomp5${B}   ${A}irc${B} ${A}pthread${B})

message(STATUS "IPP_INCLUDE_PATHS = "  ${IPP_INCLUDE_PATHS})
message(STATUS "IPPPATH = "  ${IPP_PATH})
message(STATUS "IPP_LIBS ="  ${IPP_LIBS})


############################### CUDA ################################
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR})
find_package( CUDA )

set(CUDA_VERBOSE_BUILD ON)
message(STATUS "CUDA_VERBOSE_BUILD = "  ${CUDA_VERBOSE_BUILD})
message(STATUS "CUDA_TOOLKIT_ROOT_DIR ="  ${CUDA_TOOLKIT_ROOT_DIR})


############################### Setup a subdirectory ################################

add_subdirectory(src)
