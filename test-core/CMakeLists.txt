cmake_minimum_required(VERSION 3.5)
project(test)

set(CMAKE_CXX_STANDARD 11)

add_executable(tests
        main.cpp
        delaunay/main_test_delaunay.cpp)

add_subdirectory(../siblings/gtest ./gtest)
target_link_libraries(tests gtest)
target_link_libraries(tests core)