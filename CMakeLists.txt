cmake_minimum_required (VERSION 2.8.11)
project (multirotor_sim)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

if (NOT TARGET geometry)
    add_subdirectory(lib/geometry)
    include_directories(lib/geometry/include)
endif()

if (NOT TARGET nanoflann_eigen)
    add_subdirectory(lib/nanoflann_eigen)
    include_directories(lib/nanoflann_eigen/include)
endif()

if (NOT TARGET solve_care)
  add_subdirectory(lib/solve_care)
  include_directories(lib/solve_care/include)
endif()

include_directories(include/multirotor_sim
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
    lib/nanoflann/include
)

add_library(multirotor_sim STATIC
    src/controller.cpp
    src/dynamics.cpp
    src/simulator.cpp
    src/environment.cpp
)
target_link_libraries(multirotor_sim ${YAML_CPP_LIBRARIES} stdc++fs geometry nanoflann_eigen solve_care)
