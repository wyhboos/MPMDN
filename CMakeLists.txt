cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(custom_ops)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_BUILD_TYPE Debug)
# find_package(Torch REQUIRED)

find_package(ompl REQUIRED)
include_directories(${OMPL_INCLUDE_DIRS})
link_directories(${OMPL_LIBRARY_DIRS})
# add_library(mylib SHARED cpp/src/mpn.cpp)
# target_link_libraries(mylib ${OMPL_LIBRARIES})


add_executable(main cpp/src/main.cpp)
# add_executable(test cpp/src/test.cpp)
target_link_libraries(main ${OMPL_LIBRARIES})
# set_property(TARGET test PROPERTY CXX_STANDARD 14)