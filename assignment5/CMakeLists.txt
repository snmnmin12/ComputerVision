cmake_minimum_required(VERSION 2.8)

project (CV)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package( OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

file(GLOB HEADERS "*.h")
file(GLOB SOURCES "*.cpp")
add_executable(CV ${HEADERS} ${SOURCES}) 
target_link_libraries(CV ${OpenCV_LIBS})
