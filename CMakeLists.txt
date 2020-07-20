cmake_minimum_required(VERSION 2.8)
project(color_detector)

add_compile_options(-std=c++11)

find_package(OpenCV)

include_directories(
	./include
	${OpenCV_INCLUDE_DIRS}
)


add_executable(find_color ./src/FindColor.cpp ./include/ColorDetector.cpp)
target_link_libraries(
    find_color
	${OpenCV_LIBRARIES}
)

add_executable(find_H ./src/FindH.cpp ./include/ColorDetector.cpp)
target_link_libraries(
    find_H
	${OpenCV_LIBRARIES}
)

add_executable(test ./src/test.cpp ./include/ColorDetector.cpp)
target_link_libraries(
    test
	${OpenCV_LIBRARIES}
)
