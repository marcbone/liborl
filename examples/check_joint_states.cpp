// Copyright (c) 2020 Maximilian von Unwerth
// Licensed under the EUPL-1.2-or-later
/**
 * @example check_joint_states.cpp
 * @authors Maximilian von Unwerth
 */

#include <array>
#include <iostream>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <iomanip>
#include <liborl/liborl.h>

/**
 * Watch the robots joint states to identify possible problems
 * @param argc Not used
 * @param argv Not used
 * @return Not used
 */
int main(int argc, char **argv) {
    try {
        if (argc != 2) {
            std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
            return -1;
        }
        orl::Robot r(argv[1]);
        franka::Robot &robot = r.get_franka_robot();
        // load the kinematics and dynamics model
        franka::Model model = robot.loadModel();

        std::array<double, 7> q_min = {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
        std::array<double, 7> q_max = {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};

        while (true) {
            std::cout << std::fixed << std::setprecision(4);
            std::array<double, 7> joint_states = robot.readOnce().q; //Get the robots current configuration
            std::stringstream min, max, real_values;
            min << "min: ";
            real_values << "val: ";
            max << "max: ";
            for (int i = 0; i < 7; i++) { // Iterate over all joints
                min << q_min[i] << ",\t";
                real_values << joint_states[i] << ",\t";
                max << q_max[i] << ",\t";

            }
            std::cout << min.str() << std::endl;
            std::cout << real_values.str() << std::endl;
            std::cout << max.str() << std::endl;

            std::cout << std::endl;
            r.wait_milliseconds(1000);
        }


    } catch (const franka::Exception &ex) {
        // print exception
        std::cout << ex.what() << std::endl;
    }
    return 0;
}