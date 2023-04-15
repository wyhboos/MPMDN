cmake_minimum_required(VERSION 3.0)
project(custom_ops)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_CXX_COMPILER "g++")
# set(CMAKE_CXX_FLAGS "-O2 -std=c++11")


find_package(Torch REQUIRED)
find_package(ompl REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(Boost COMPONENTS python REQUIRED)

# find_package(Boost REQUIRED COMPONENTS system random)


include_directories(${OMPL_INCLUDE_DIRS} ${TORCH_INCLUDE_DIRS} /home/wyh/Code/MPMDN/cpp/include)
link_directories(/home/wyh/Code/MPMDN/build)
add_library(mylib SHARED cpp/src/mpn.cpp cpp/src/mpmdn.cpp)
target_link_libraries(mylib PUBLIC ${OMPL_LIBRARIES} ${TORCH_LIBRARIES} cnpy)

set(CMAKE_SHARED_MODULE_PREFIX "")
add_executable(main cpp/src/main.cpp)
# add_executable(test cpp/src/test_eigenmvn.cpp)
# target_link_libraries(test ${Boost_LIBRARIES})

# add_executable(test cpp/src/test.cpp)
target_link_libraries(main mylib ${OMPL_LIBRARIES})
# target_link_libraries(main PRIVATE -Wl,--start-group ${OMPL_LIBRARIES} ${TORCH_LIBRARIES} -Wl,--end-group

# set_property(TARGET main PROPERTY CXX_STANDARD 14)


add_library(MPN MODULE cpp/src/bind.cpp)
# # Set up the libraries and header search paths for this target
target_link_libraries(MPN ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} mylib)
target_include_directories(MPN PRIVATE ${PYTHON_INCLUDE_DIRS})