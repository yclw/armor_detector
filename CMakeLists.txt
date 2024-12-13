cmake_minimum_required(VERSION 3.5)
project(quick_discovery)

# 优化
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3 -march=native -fopenmp")

# opencv
find_package(OpenCV REQUIRED)
find_package(TBB REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(include)

add_executable(quick_discovery
    test/main.cpp
    src/armor_detector.cpp
    src/number_classifier.cpp
)

target_link_libraries(quick_discovery ${OpenCV_LIBS} TBB::tbb /home/yc/my_sdk_test01/lib/libMVSDK.so)