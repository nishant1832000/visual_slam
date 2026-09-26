
#include <gflags/gflags.h>
#include "orb_slam/visual_odometry.h"


DEFINE_string(config_file, "/home/nishant/orb_slam_practice/config/default.yaml", "config file path");

int main(int argc, char **argv) {
    google::ParseCommandLineFlags(&argc, &argv, true);

    orb_slam::VisualOdometry::Ptr vo(
        new orb_slam::VisualOdometry(FLAGS_config_file));
    assert(vo->Init() == true);
    vo->Run();

    return 0;
}