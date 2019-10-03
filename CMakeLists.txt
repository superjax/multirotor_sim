cmake_minimum_required (VERSION 2.8.11)
project (multirotor_sim)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(LAPACK REQUIRED)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/findDeps.cmake)

find_deps(geometry https://github.com/superjax/geometry)
find_deps(gnss_utils https://github.com/superjax/gnss_utils)
find_deps(nanoflann_eigen https://github.com/superjax/nanoflann_eigen)
find_deps(lin_alg_tools https://github.com/superjax/lin_alg_tools)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${lin_alg_tools_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
    ${gnss_utils_INCLUDE_DIRS}
    ${nanoflann_eigen_INCLUDE_DIRS}
    ${geometry_INCLUDE_DIRS})

add_library(multirotor_sim SHARED
    src/controller.cpp
    src/dynamics.cpp
    src/simulator.cpp
    src/environment.cpp
)
target_link_libraries(multirotor_sim
    stdc++fs
    ${YAML_CPP_LIBRARIES}
    ${LAPACK_LIBRARIES}
    gnss_utils)

option(BUILD_TEST "Build Tests" OFF)
if (BUILD_TEST)
    message(STATUS "BUILDING TESTS")
    find_package(GTest REQUIRED)
    find_package(Boost REQUIRED COMPONENTS system thread)
    add_definitions(-DMULTIROTOR_SIM_DIR="${CMAKE_CURRENT_LIST_DIR}")
    include_directories(include ${GTEST_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
    add_executable(multirotor_sim_test
        src/test/test_raw_gnss.cpp
        src/test/test_state.cpp
        src/test/test_dynamics.cpp
        src/test/test_reference_controller.cpp
        )
    target_link_libraries(multirotor_sim_test ${GTEST_LIBRARIES} gtest_main gtest pthread multirotor_sim)
else()
    message (STATUS "SKipping TESTS")
endif()

# install
set(LIB_DEST lib/${PROJECT_NAME})
set(INCLUDE_DEST include)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION ${LIB_DEST}
  LIBRARY DESTINATION ${LIB_DEST}
)
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${INCLUDE_DEST}
  FILES_MATCHING PATTERN "*.h"
)

# install CMake package configuration
install(EXPORT ${PROJECT_NAME}-targets DESTINATION ${LIB_DEST})
install(FILES ${PROJECT_NAME}-config.cmake DESTINATION ${LIB_DEST})
