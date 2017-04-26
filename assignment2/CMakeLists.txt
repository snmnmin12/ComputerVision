cmake_minimum_required(VERSION 2.8)
project (ComputerVision)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin/")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package( OpenCV REQUIRED)

#if the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of themmessage(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

include_directories(${OpenCV_INCLUDE_DIRS})
file(GLOB HEADERS "*.h")
#file(GLOB SOURCES "*.cpp")


#test2
add_executable(CV2 test2.cpp ${HEADERS})
target_link_libraries(CV2 ${OpenCV_LIBS})

#test3
add_executable(CV3 test3.cpp ${HEADERS})
target_link_libraries(CV3 ${OpenCV_LIBS})

