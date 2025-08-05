cmake_minimum_required(VERSION 3.5)
project(Analyzer)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/src
)
file(GLOB all_SRCS
  "${PROJECT_SOURCE_DIR}/include/*.h"
  "${PROJECT_SOURCE_DIR}/include/*.hpp"
  "${PROJECT_SOURCE_DIR}/src/*.cpp"
  "${PROJECT_SOURCE_DIR}/src/*.c"
)
add_executable(Analyzer ${all_SRCS})

set("OpenCV_DIR" "${CMAKE_CURRENT_SOURCE_DIR}/libs/opencv/build")
find_package(OpenCV REQUIRED PATHS ${OpenCV_DIR} NO_DEFAULT_PATH)
if (NOT OpenCV_FOUND)
  message(FATAL_ERROR "OpenCV not found. Please set OpenCV_DIR to the correct path.")
endif()
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(Analyzer ${OpenCV_LIBS})

set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost REQUIRED COMPONENTS filesystem)

set("PCL_DIR" "${CMAKE_CURRENT_SOURCE_DIR}/libs/pcl/install")
find_package(PCL REQUIRED PATHS ${PCL_DIR} NO_DEFAULT_PATH)
# find_package(PCL REQUIRED)
if(NOT PCL_FOUND)
  message(FATAL_ERROR "PCL not found. Please set PCL_DIR to the correct path.")
endif()
message(STATUS "PCL include directories: ${PCL_INCLUDE_DIRS}")
include_directories(${PCL_INCLUDE_DIRS})
message(STATUS "PCL libraries: ${PCL_LIBRARIES}")
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(Analyzer ${PCL_LIBRARIES})

if(MSVC)
  target_compile_options(Analyzer PRIVATE /W4 /WX)
else()
  target_compile_options(Analyzer PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()