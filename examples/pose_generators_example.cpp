// Copyright (c) 2020 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
/**
 * @example pose_generators_example.cpp
 * @brief example which shows the capabilites of PoseGenerators
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
    Pose start_p = robot.get_current_pose();
    start_p.set_position(0.45, -0.1, 0.5);
    robot.cart_motion(start_p, 3);
    Position circle_center = Position(0.45, -0.1, 0.39);
    Orientation down_orientation;
    down_orientation.set_RPY(-3 * M_PI / 8, 0, 0);
    robot.line_gripper_up_to_orientation(down_orientation, 2);


    // TCP Motion
    auto tcp_motion = PoseGenerators::RelativeMotion(Position(0, 0, 0.2), Frame::RobotTCP);
    apply_speed_profile(tcp_motion);
    robot.move_cartesian(tcp_motion, 4);

    tcp_motion = PoseGenerators::RelativeMotion(Position(0, 0, -0.2), Frame::RobotTCP);
    apply_speed_profile(tcp_motion);
    robot.move_cartesian(tcp_motion, 4);

    // line up gripper
    down_orientation.set_RPY(-M_PI, 0, 0);
    robot.line_gripper_up_to_orientation(down_orientation, 2);


    // screw motion
    auto no_rotation = generate_constant_orientation_orientation_generator();
    auto circle = PoseGenerators::CirclePoseGenerator(circle_center, -M_PI * 2, Plane::YZ, no_rotation);
    auto move_horizontal = PoseGenerators::RelativeMotion(Position(0.2, 0, 0.), Frame::UnitFrame);
    auto pose_generator = move_horizontal * circle;
    apply_speed_profile(pose_generator);
    robot.move_cartesian(pose_generator, 5);

}

