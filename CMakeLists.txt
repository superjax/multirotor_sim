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
endif()

if (NOT TARGET nanoflann_eigen)
    add_subdirectory(lib/nanoflann_eigen)
endif()

if (NOT TARGET lin_alg_tools)
  add_subdirectory(lib/lin_alg_tools)
endif()

if (NOT TARGET gnss_utils)
    add_subdirectory(lib/gnss_utils)
endif()

add_library(multirotor_sim
    src/controller.cpp
    src/dynamics.cpp
    src/simulator.cpp
    src/environment.cpp
)
target_include_directories(multirotor_sim PUBLIC
    include
    lib/nanoflann/include
    lib/geometry/include)
target_link_libraries(multirotor_sim
    ${YAML_CPP_LIBRARIES}
    stdc++fs
    geometry
    nanoflann_eigen
    lin_alg_tools
    gnss_utils)

if (${GTEST_FOUND})
    add_definitions(-DMULTIROTOR_SIM_DIR="${CMAKE_CURRENT_LIST_DIR}")
    include_directories(include ${GTEST_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
    add_executable(multirotor_sim_test
        src/test/test_raw_gnss.cpp
        src/test/test_state.cpp
        src/test/test_dynamics.cpp
        src/test/test_reference_controller.cpp
        )
    target_link_libraries(multirotor_sim_test ${GTEST_LIBRARIES} gtest_main gtest pthread multirotor_sim)
endif()
