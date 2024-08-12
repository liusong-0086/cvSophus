cmake_minimum_required(VERSION 3.8)
project(cvSophus)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(THIRD_PARTY_DIR "${PROJECT_SOURCE_DIR}/thirdparty")
set(OpenCV_DIR "${THIRD_PARTY_DIR}/opencv/build/x64/vc16/lib")


find_package(OpenCV REQUIRED)

add_subdirectory(test)