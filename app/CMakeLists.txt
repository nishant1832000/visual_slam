add_executable(run_kitti run_kitti.cpp)
target_link_libraries(run_kitti orb_slam ${THIRD_PARTY_LIBS})