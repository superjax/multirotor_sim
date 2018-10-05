cmake_minimum_required (VERSION 2.8.11)
project (multirotor_sim)

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

if (NOT TARGET geometry)
    add_subdirectory(lib/geometry)
    include_directories(lib/geometry/include)
endif()

include_directories(include    
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
)

add_library(framework
    src/controller.cpp
    src/dynamics.cpp
    src/simulator.cpp
    src/environment.cpp
)
target_link_libraries(framework vi_ekf math_helper ${YAML_CPP_LIBRARIES} stdc++fs)
