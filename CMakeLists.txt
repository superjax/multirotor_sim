cmake_minimum_required (VERSION 2.8.11)
project (multirotor_sim)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(GTest)

if (NOT TARGET geometry)
    add_subdirectory(lib/geometry)
    include_directories(lib/geometry/include)
endif()

if (NOT TARGET nanoflann_eigen)
    add_subdirectory(lib/nanoflann_eigen)
    include_directories(lib/nanoflann_eigen/include)
endif()

if (NOT TARGET lin_alg_tools)
  add_subdirectory(lib/lin_alg_tools)
  include_directories(lib/lin_alg_tools/include)
endif()


include_directories(include/multirotor_sim
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
    lib/nanoflann/include
    lib/geometry/include
)

add_library(multirotor_sim STATIC
    src/controller.cpp
    src/dynamics.cpp
    src/simulator.cpp
    src/environment.cpp
    src/gtime.cpp
    src/datetime.cpp
    src/satellite.cpp
    src/gnss.cpp
)
target_link_libraries(multirotor_sim ${YAML_CPP_LIBRARIES} stdc++fs geometry nanoflann_eigen lin_alg_tools)


if (${GTEST_FOUND})
    include_directories(include ${GTEST_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
    add_executable(multirotor_sim_test
        src/test/test_gps.cpp
        src/test/test_time.cpp
        src/test/test_ephemeris.cpp
        src/test/test_raw_gps.cpp
        src/test/test_state.cpp
        src/test/test_dynamics.cpp
        src/test/test_reference_controller.cpp
        src/test/reference_algorithms.cpp
        )
    target_link_libraries(multirotor_sim_test ${GTEST_LIBRARIES} gtest_main gtest pthread multirotor_sim)
endif()
