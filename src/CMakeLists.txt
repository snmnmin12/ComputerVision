
find_package( OpenCV REQUIRED)

#if the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of themmessage(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

include_directories(${OpenCV_INCLUDE_DIRS})

#include_directories(include)
#file(GLOB SOURCES "${PROJECT_SOURCE_DIR}/src/tutorial.cpp")
#add_executable(Tu ${SOURCES})
add_executable(CV test1.cpp)
target_link_libraries(CV ${OpenCV_LIBS})

#test2
add_executable(CV2 test2.cpp)
target_link_libraries(CV2 ${OpenCV_LIBS})

#challenge
add_executable(CV3 test2.cpp)
target_link_libraries(CV3 ${OpenCV_LIBS})

#target_compile_features(Tu PRIVATE cxx_range_for)
