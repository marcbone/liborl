// Copyright (c) 2020 Maximilian von Unwerth
// Licensed under the EUPL-1.2-or-later
/**
 * @file Payload.h
 * @brief Payload representation for different types of payload.
 * @authors Maximilian von Unwerth, Marco Boneberger
 */
#ifndef LIBORL_PAYLOAD_H
#define LIBORL_PAYLOAD_H

#include <array>

/**
 * Class which represents a grasped payload
 */
namespace orl {
    struct Payload {
        /**
         * Payload mass
         */
        double mass;

        /**
         * Flange to CoM of payload
         */
        std::array<double, 3> pos_wrt_flange;

        /// Inertia matrix
        std::array<double, 9> inertia_matrix;
    };
    namespace Payloads {

        /**
         * Creates a Payload which is shaped like a cylinder
         * @param radius of the cylinder in meter
         * @param height of the cylinder in meter
         * @param mass of the cylinder in kg
         * @param position_wrt_flange translation vector from the robot flange to the center of mass of the cylinder.
         *
         * \note
         * Often you just want to have a z offset for the translation
         * @return a Payload with the desired properties of a cylinder
         */
        Payload Cylinder(double radius, double height, double mass, const std::array<double, 3> &position_wrt_flange);

        /**
         * Creates a Payload which is shaped like a sphere
         * @param radius in meter
         * @param mass in kg
         * @param position_wrt_flange translation vector from the robot flange to the center of mass of the sphere.
         *
         * \note
         * Often you just want to have a z offset for the translation
         * @return a Payload with the desired properties of a spherical object
         */
        Payload Sphere(double radius, double mass, const std::array<double, 3> &position_wrt_flange);
    }
}
#endif //LIBORL_PAYLOAD_H
