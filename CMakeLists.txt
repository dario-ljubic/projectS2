cmake_minimum_required(VERSION 3.9)
project(projectS2)

set(CMAKE_CXX_STANDARD 11)

set(OPENCV_DIR libs_external/opencv-3.4.1/build)
find_package(OpenCV REQUIRED)

#Since Eigen is just a collection of headers, it is possible to just
# include it as it is
include_directories(
        include
        libs_external/eigen-eigen-5a0156e40feb/Eigen
        ${OpenCV_INCLUDE_DIRS})

#The file() command is used to add the source files to the project.
#GLOB (or GLOB_RECURSE) is used to create a list of all of the files
#that meet the globbing expression (i.e., “src/*.cpp“) and add them to
#a variable SOURCES.
file(GLOB SOURCES "src/*.cpp")

add_executable(${PROJECT_NAME} ${SOURCES})

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})