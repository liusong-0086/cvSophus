add_executable(test_so3 test_so3.cc)
target_include_directories(test_so3 PUBLIC
    "../src"
    ${OPENCV_INCLUDES}
)

target_link_libraries(test_so3 PUBLIC
    ${OpenCV_LIBS}
)

install(TARGETS test_so3 OPTIONAL)

add_executable(test_se3 test_se3.cc)
target_include_directories(test_se3 PUBLIC
    "../src"
    ${OPENCV_INCLUDES}
)

target_link_libraries(test_se3 PUBLIC
    ${OpenCV_LIBS}
)

install(TARGETS test_se3 OPTIONAL)