// Copyright (c) 2020 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
/**
 * @example s_curve_example.cpp
 * @brief shows how to use an S-Curve SpeedProfile
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
    robot.absolute_cart_motion(0.65, 0.0, 0.2, 2);
    Orientation down_orientation(-M_PI, 0, 0);
    robot.line_gripper_up_to_orientation(down_orientation, 1);
    std::vector<Position> way_points{{-0.20, 0.4, 0.5},
                                     {0.3,   0.4, 0.5},
                                     {0.3,   0.4, 0.2}};
    auto circle_movement = PoseGenerators::CirclePoseGenerator({0.55, 0, 0}, 10 * M_PI, Plane::XY,
                                                               generate_constant_orientation_orientation_generator());
    double trajectory_duration;
    const double max_jerk = 16;
    const double max_acceleration = 6;
    const double max_velocity = 0.1;
    apply_speed_profile(circle_movement,
                        SpeedProfiles::SCurveProfile(circle_movement, robot.get_current_pose().getPosition(), max_jerk,
                                                     max_acceleration,
                                                     max_velocity,
                                                     trajectory_duration));
    std::cout << trajectory_duration << std::endl;
    robot.move_cartesian(circle_movement, trajectory_duration);
}

