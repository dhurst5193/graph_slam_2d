add_library(graph_slam_2d SHARED
    robot_interface.cc
    laser_frame.cc
    scan_matcher.cc
)
target_link_libraries( graph_slam_2d
    ${THIRD_PARTY_LIBS}
)
