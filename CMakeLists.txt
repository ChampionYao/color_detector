cmake_minimum_required(VERSION 2.8)
project(color_detector)

add_compile_options(-std=c++11)

find_package(OpenCV)

include_directories(
	${OpenCV_INCLUDE_DIRS}
)


add_executable(test ./test.cpp)
target_link_libraries(
    test
	${OpenCV_LIBRARIES}
)

