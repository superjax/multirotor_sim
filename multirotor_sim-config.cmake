
get_filename_component(SELF_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
include(${SELF_DIR}/multirotor_sim-targets.cmake)
get_filename_component(aysnc_comm_INCLUDE_DIRS "${SELF_DIR}/../../include/multirotor_sim ${SELF_DIR}/../../lib/lin_alg_tools ${SELF_DIR}/../../lib/nanoflann_eigen" ABSOLUTE)
set(multirotor_sim_LIBRARIES multirotor_sim nanoflann_eigen lin_alg_tools)