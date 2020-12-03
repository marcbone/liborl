// Copyright (c) 2020 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
/**
 * @example force_stop_example.cpp
 * @brief The robot stops the movement when there is an object in the way
 */

#include <liborl/liborl.h>
#include <memory>

int main(int argc, char **argv) {


    try {
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

        orl::Pose pregrasp_pose(robot.get_current_pose());
        pregrasp_pose.set_position(0.4, 0.15, 0.05);
        pregrasp_pose.set_RPY(-M_PI, 0, 0);

        orl::Pose move_pose(robot.get_current_pose());
        move_pose.set_position(0.7, 0.15, 0.05);
        move_pose.set_RPY(-M_PI, 0, 0);

        robot.close_gripper();
        robot.cart_motion(pregrasp_pose, 5);
        robot.cart_motion(move_pose, 5, orl::StopConditions::Force(5));
        robot.cart_motion(pregrasp_pose, 5);
        robot.joint_motion(q_goal, 0.2);

    } catch (const franka::Exception &e) {
        std::cout << e.what() << std::endl;
        return -1;
    }
    return 0;
}