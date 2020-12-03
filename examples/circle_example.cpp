// Copyright (c) 2020 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
/**
 * @example circle_example.cpp
 * @brief shows how to use the make circle Movements
 */
#include <iostream>
#include <liborl/liborl.h>

using namespace std;

int main(int argc, char **argv) {


    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }
    orl::Robot robot(argv[1]);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Double tap the robot to continue..." << std::endl;
    robot.double_tap_robot_to_continue();

    std::array<double, 7> q_goal = {{0.542536, 0.258638, -0.141972, -1.99709, -0.0275616, 2.38391, 1.12856}};
    robot.joint_motion(q_goal, 0.2);

    orl::Pose start_p = robot.get_current_pose();
    start_p.set_position(0.45, -0.1, 0.5);
    robot.cart_motion(start_p, 3);
    orl::Position circle_center = orl::Position(0.45, -0.1, 0.39);
    orl::Orientation down_orientation;
    down_orientation.set_RPY(-M_PI, 0, 0);

    robot.line_gripper_up_to_orientation(down_orientation, 2);

    auto no_rotation = orl::generate_constant_orientation_orientation_generator();
    robot.move_circle_yz(circle_center, -M_PI * 2, 5, no_rotation);
    robot.move_circle_xy(orl::Position(0.4, 0, 0.4), -M_PI * 2, 5, no_rotation);
    robot.move_circle_zx(circle_center, -M_PI * 2, 5, no_rotation);
    robot.move_circle_yz(circle_center, -M_PI_2, 5);
    robot.move_circle_xy(circle_center, -M_PI_2 * 0.5, 2);
    robot.move_circle_yz(circle_center, M_PI_2, 5);
    robot.move_circle_zx(circle_center, -M_PI_2 * 0.5, 10);
}