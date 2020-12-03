// Copyright (c) 2020 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
/**
 * @file enums.h
 * @brief File which contains some Enums needed for selecting Axis and Planes
 * @authors Marco Boneberger
 */

#ifndef LIBORL_ENUMS_H
#define LIBORL_ENUMS_H
namespace orl {


/**
 * Axis enum for selecting an axis of the robot
 */
    enum Axis {
        X = 0, Y = 1, Z = 2
    };
/**
    * Plane enum for selecting XY, YZ, ZX plane
    */
    enum class Plane {
        XY, YZ, ZX
    };
}
#endif //LIBORL_ENUMS_H
