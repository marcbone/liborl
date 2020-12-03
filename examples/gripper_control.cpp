// Copyright (c) 2020 Maximilian von Unwerth
// Licensed under the EUPL-1.2-or-later
/**
 * @example gripper_control.cpp
 * @authors Maximilian von Unwerth
 *
 * Controls Franka Emika Grippers - designed for a 4 arm platform
 * Simply enter the number of the robot and o or c for open and close
 */
#include <iostream>
#include <liborl/liborl.h>
#include <string>

using namespace std;


/**
 * Open or close the gripper using command line instructions
 * @param argc Not used
 * @param argv Not used
 * @return Not used
 */
int main(int argc, char **argv) {


    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }

    try {
        orl::Robot robot(argv[1]);
        string input;
        while (input != "o" and input != "c") { // wait for valid input
            cout << "Enter (o)PEN or (c)LOSE to control the gripper." << endl;
            cin >> input;
        }
        if (input == "o") {
            robot.open_gripper(0.05);
        }
        if (input == "c") {
            robot.close_gripper();
        }
        return 0;
    } catch (const franka::Exception &ex) {
        std::cout << ex.what() << std::endl; // print exception details
    }
}