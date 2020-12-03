// Copyright (c) 2020 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
/**
 * @file StopCondition.h
 * @brief defines StopCondtions and offers a method to stop the robot by Force
 * @authors Marco Boneberger
 */

#ifndef LIBORL_STOPCONDITION_H
#define LIBORL_STOPCONDITION_H

#include <liborl/PoseGenerator.h>

namespace orl {
/**
 * Function which returns a bool whether to stop the robot at a specific condition
 */
    typedef std::function<bool(const PoseGeneratorInput &)> StopCondition;
    namespace StopConditions {

/**
         * generates a function which stops the robot when there is a force on the endeffector which exceeds max_force
         * and the movement within the last part of the movement
         * \warning if you dont set a minimum progress the robot can abort the motion at any time due to the condition.
         * if you open the gripper afterwards the object could fall at anytime!
         * @param max_force
         * @param minimum_progress minimum progress [0,1] after which the stop condition can be fired
         * @return a StopCondition
         */
        StopCondition Force(double max_force, double minimum_progress = 0);

    }
}
#endif //LIBORL_STOPCONDITION_H
