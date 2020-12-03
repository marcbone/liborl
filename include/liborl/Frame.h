// Copyright (c) 2020 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
/**
 * @file Frame.h
 * @brief defines the Frame struct
 * @authors Marco Boneberger
 */
#ifndef LIBORL_FRAME_H
#define LIBORL_FRAME_H


/// enum which is there to select a Frame of reference. When in doubt use RobotBase
enum class Frame {
    RobotBase, ///< The the end-effector Frame wrt to the Robot Base. This is what you want in most cases
    RobotTCP, ///< This is the end-effector Frame of the robot
    UnitFrame  ///< Frame of the Unit-Homogenous Matrix (Zero rotation around any axis and zero translation)
};
#endif //LIBORL_FRAME_H
