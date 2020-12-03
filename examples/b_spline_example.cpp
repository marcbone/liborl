// Copyright (c) 2020 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
/**
 * @example b_spline_example.cpp
 * @brief an example on showing how to use B-Splines
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
    robot.absolute_cart_motion(-0.2, 0.4, 0.2, 2);
    Orientation down_orientation;
    down_orientation.set_RPY(-M_PI, 0, 0);
    robot.line_gripper_up_to_orientation(down_orientation, 1);

    // the first waypoint will always be the robot position, you dont have to specify this one
    std::vector<Position> way_points{{-0.20, 0.4, 0.5},
                                     {0.3,   0.4, 0.5},
                                     {0.3,   0.4, 0.2}};
    const int degree_of_b_spline = 2;
    auto b_spline_movement = PoseGenerators::BSplineMotion(way_points, degree_of_b_spline);
    double trajectory_duration;
    const double max_jerk = 16;
    const double max_acceleration = 6;
    const double max_velocity = 0.1;
    apply_speed_profile(b_spline_movement,
                        SpeedProfiles::SCurveProfile(b_spline_movement, robot.get_current_pose().getPosition(),
                                                     max_jerk, max_acceleration,
                                                     max_velocity,
                                                     trajectory_duration));
    std::cout << trajectory_duration << std::endl;
    robot.move_cartesian(b_spline_movement, trajectory_duration);
}

