// Copyright (c) 2020 Maximilian von Unwerth
// Licensed under the EUPL-1.2-or-later
/*
 * Payload representation for different types of payload.
 * The inertia matrix of the payloads is calculated by its type
 */
#include <liborl/Payload.h>
using namespace orl;
Payload Payloads::Sphere(double radius, double mass, const std::array<double, 3> &position_wrt_flange) {
    double i = (2.0 / 5.0) * mass * radius * radius;
    std::array<double, 9> inertia_matrix = {i, 0, 0, 0, i, 0, 0, 0, i};
    return {mass, position_wrt_flange, inertia_matrix};
}

Payload Payloads::Cylinder(double radius, double height, double mass,
                           const std::array<double, 3> &position_wrt_flange) {
    std::array<double, 9> inertia_matrix = {(mass / 12) * (3 * radius * radius + height * height), 0, 0, 0,
                                            (mass / 12) * (3 * radius * radius + height * height), 0, 0, 0,
                                            (mass / 2) * radius * radius};

    return {mass, position_wrt_flange, inertia_matrix};
}
