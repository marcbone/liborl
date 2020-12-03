// Copyright (c) 2020 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
/**
 * @file geometry_math.h
 * @brief set of useful functions to do geometry
 * @authors Marco Boneberger
 */

#ifndef LIBORL_GEOMETRY_MATH_H
#define LIBORL_GEOMETRY_MATH_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>

/**
 * calculates the angle between a direction "a" and a direction "b" around a given normal
 * @param a from direction from origin
 * @param b to direction from origin
 * @param normal normal around which to calculate the angle
 * @return angle in radian
 */
double angle_between(const Eigen::Vector3d &a, const Eigen::Vector3d &b, const Eigen::Vector3d &normal);

/**
 * Function to calculate a circle generator around a center
 *
 * This function returns a function where you can pass in a value [0,1] and it spits out a point on the circle
 * where 0 is the start and 1 is at the end.
 *
 * @param center center of the circle
 * @param normal normal between the moving direction and the direction to the center
 * @param start start position of the circle
 * @param rotation_angle how many radian you want on the circle
 * @return parameterized function which generates the circle
 */
std::function<Eigen::Vector3d(double progress)>
generate_circle_movement(const Eigen::Vector3d &center, const Eigen::Vector3d &normal, const Eigen::Vector3d &start,
                         double rotation_angle);

/**
 * generates a circle function on the x-y-plane.
 * the center gets automatically projected to the z position of the center
 * \see generate_circle_movement for more details
 * @param center center of the circle
 * @param start start position of the circle
 * @param rotation_angle how many radian you want on the circle
 * @return circle generating function
 */
std::function<Eigen::Vector3d(double progress)>
generate_circle_movement_xy_plane(const Eigen::Vector3d &center, const Eigen::Vector3d &start, double rotation_angle);

/**
 * generates a circle function on the y-z-plane.
 * the center gets automatically projected to the x position of the center
 * \see generate_circle_movement for more details
 * @param center center of the circle
 * @param start start position of the circle
 * @param rotation_angle how many radian you want on the circle
 * @return circle generating function
 */
std::function<Eigen::Vector3d(double progress)>
generate_circle_movement_yz_plane(const Eigen::Vector3d &center, const Eigen::Vector3d &start, double rotation_angle);

/**
 * generates a circle function on the z-x-plane.
 * the center gets automatically projected to the y position of the center
 * \see generate_circle_movement for more details
 * @param center center of the circle
 * @param start start position of the circle
 * @param rotation_angle how many radian you want on the circle
 * @return circle generating function
 */
std::function<Eigen::Vector3d(double progress)>
generate_circle_movement_zx_plane(const Eigen::Vector3d &center, const Eigen::Vector3d &start, double rotation_angle);


/**
 * generates a Bezier-Curve-function which maps [0,1] -> Positions
 * @param points vector of control points
 * @return Bezier Function
 */
std::function<Eigen::Vector3d(double progress)>
generate_bezier_curve(const std::vector<Eigen::Vector3d> &points);


/**
 * calculates the value of a B-spline with known in index k.
 * This is an mostly internal function. You will probably better of with the generate_spline function
 * This is the optimized variant of De Boors algorithm according to <a href="https://en.wikipedia.org/wiki/De_Boor's_algorithm#Optimizations">Wikipedia</a>
 *  and its <a href="https://en.wikipedia.org/wiki/De_Boor's_algorithm#Example_implementation">Python implementation</a>
 * @param k Index of knot interval that contains x
 * @param p Degree of B-spline
 * @param x Position
 * @param t Vector of knot positions, needs to be padded
 * @param c Vector of control points
 * @return Value of the b-Spline
 */
Eigen::Vector3d
b_spline_de_boor(int k, int p, double x, const std::vector<int> &t, const std::vector<Eigen::Vector3d> &c);

/**
 * generates a B-Spline function which maps [0,1] -> Positions using De Boors Algorithm
 * @param points vector of control points
 * @param degree degree of the B-Spline
 * @param knot_vector Knot vector see <a href="https://en.wikipedia.org/wiki/Non-uniform_rational_B-spline#Technical_specifications">Wikipedia:NURBS N</a> for an explanation of knot vectors
 * @return B-Spline function
 */
std::function<Eigen::Vector3d(double progress)>
generate_b_spline(const std::vector<Eigen::Vector3d> &points, int degree, const std::vector<int> &knot_vector);

/**
 * generates a B-Spline function which maps [0,1] -> Positions using De Boors Algorithm
 * with a default knot vector
 *
 * @param points vector of control points
 * @param degree degree of the B-Spline
 * @return B-Spline function
 */
std::function<Eigen::Vector3d(double progress)>
generate_b_spline(const std::vector<Eigen::Vector3d> &points, int degree);

/**
 * So you wanna use B-Splines? But you don't know anything 'bout knot vectors, don't ya? Dont you worry. We got you covered.
 * This function generates sane knot vectors for any amount of control points and degrees.
 *
 * For degree 1 it creates a natural sequence [0,1,2,..,m]
 * For degree 2 it creates something like [0,0,0,1,2,3,..,m-2,m-1,m,m,m]
 * and for Further degrees it creates an additional 0 at the start and m at the end
 * @param num_control_points amount of control points
 * @param degree degree of the B-Spline
 * @return knot vector
 */
std::vector<int> create_default_knot_vector(int num_control_points, int degree);

#endif //LIBORL_GEOMETRY_MATH_H
