// Copyright (c) 2020 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
/**
 * @example bezier_example.cpp
 *
 */

#include <liborl/liborl.h>
#include <memory>


using namespace orl;

int main(int argc, char **argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }
   Robot robot(argv[1]);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Double tap the robot to continue..." << std::endl;
    robot.double_tap_robot_to_continue();
    std::array<double, 7> q_goal = {{0.542536, 0.258638, -0.141972, -1.99709, -0.0275616, 2.38391, 1.12856}};
    robot.joint_motion(q_goal, 0.2);
    std::vector<Position> way_points{{-0.20, 0.2, 0.8},
                                     {0.3,   0.2, 0.8},
                                     {0.3,   0.2, 0.2}};
    auto bezier_movement = PoseGenerators::BezierMotion(way_points);
    apply_speed_profile(bezier_movement, SpeedProfiles::QuinticPolynomialProfile());
    const double execution_time = 10;
    robot.move_cartesian(bezier_movement, execution_time);
}

