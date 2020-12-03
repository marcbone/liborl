// Copyright (c) 2020 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
/**
 * @file PoseGenerator.h
 * @brief defines PoseGenerators and contains a set of default PoseGenerators
 * @authors Marco Boneberger
 */

#ifndef LIBORL_POSEGENERATOR_H
#define LIBORL_POSEGENERATOR_H


#include <franka/robot_state.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <liborl/Pose.h>
#include <boost/optional.hpp>
#include <liborl/Frame.h>
#include <liborl/enums.h>

namespace orl {

/**
 * Input for the Generator Functions
 */
    struct PoseGeneratorInput {
        double progress; ///< progress of the movement between 0 and 1
        orl::Pose initial_pose; ///< initial pose at the start of the movement
        franka::RobotState state; ///< the current state of the robot
    };
/**
 * Functions which generates Positions as Eigen::Vector3d.
 * The function takes a PoseGeneratorInput and spits out a Eigen::Vector3d.
 */
    typedef std::function<Eigen::Vector3d(const PoseGeneratorInput &)> PositionGenerator;
/**
 * Functions which generates Orientations as Eigen::Quaterniond.
 * The function takes a PoseGeneratorInput and spits out a Eigen::Quaterniond.
 */
    typedef std::function<Eigen::Quaterniond(const PoseGeneratorInput &)> OrientationGenerator;

/**
 * Functions which generates Poses as orl::Pose.
 * The function takes a PoseGeneratorInput and spits out a orl::Pose.
 */
    typedef std::function<orl::Pose(const PoseGeneratorInput &)> PoseGenerator;

/**
 * A SpeedProfile is a function which takes a double which represents the time-wise progress of
 * a motion as interval of [0,1] and converts it to a motion-wise progress as interval of [0,1]
 *
 * While the the time-wise progress of the motion is always linear (with time), the motion progress
 * can be any monotonic function of [0,1] -> [0,1]
 */
    typedef std::function<double(double)> SpeedProfile;

/**
 * Generates a PoseGenerator by combining a Position- and a OrientationGenerator
 * @param position_generator function which generates positions
 * @param orientation_generator  function which generates orientations
 * @param speed_profile optional speed profile which should be applied. If no SpeedProfile is specified the
 * QuinticPolynomialProfile will be used. If you do not want that use the the LinearProfile.
 * @return a PoseGenerator
 */
    PoseGenerator
    generate_pose_generator(const PositionGenerator &position_generator,
                            const OrientationGenerator &orientation_generator,
                            const boost::optional<SpeedProfile> &speed_profile = boost::none);

/**
 * creates an orientation generator where the robot does not change its orientation.
 * This is useful when you only want the robot to move but do not want the end-effector orientation to change
 * @return orientation generator which does not alter the robots orientation
 */
    OrientationGenerator generate_constant_orientation_orientation_generator(
            const boost::optional<Eigen::Quaterniond> &orientation = boost::none);

/**
 * creates an orientation generator where the robot orients itself towards a given point.
 * @param angle_axis point where the end-effector should point to
 * @return
 */
    OrientationGenerator generate_angle_axis_orientation_generator(const Eigen::AngleAxisd &angle_axis);

/**
 * creates a position generator where the robot does not change its position.
 * Useful when you only want to control the orientation of the robot
 * @return PositionGenerator which does not alter the robots position
 */
    PositionGenerator
    generate_constant_position_position_generator(const boost::optional<Position> &position = boost::none);

/**
 * creates a PoseGenerator which interpolates from the start position of the robot to the desired goal pose
 * @param end_pose desired goal pose
 * @return PoseGenerator function to interpolate to the goal pose
 */
    PoseGenerator generate_pose_interpolation_pose_generator(const orl::Pose &end_pose);

/**
 * creates a orientation generator which interpolates between orientations via slerp.
 * @param end_orientation  desired goal orientation
 * @return orientation generator function to interpolate to the goal orientation
 */
    OrientationGenerator
    generate_orientation_interpolation_orientation_generator(const Eigen::Quaterniond &end_orientation);


    PoseGenerator operator*(const PoseGenerator &lhs, const PoseGenerator &rhs);

    namespace PoseGenerators {
        /**
         * Generates a PoseGenerator which describes a Circle
         * @param center center of the circle
         * @param rotation_angle the length of the circle in radians. This can be higher than 2 * pi for more multiple circles
         * @param plane on which plane the circle should be performed
         * @param maybe_orientation_generator an orientation generator for the movement. If there is no orientation generator. this function
         * will generate one which rotates the end-effector around the normal of the plane
         * @return a PoseGenerator which describes a circle
         */
        PoseGenerator CirclePoseGenerator(const Position &center, double rotation_angle, const Plane &plane,
                                          const boost::optional<OrientationGenerator> &maybe_orientation_generator = boost::none);

        /**
         * Generates a PoseGenerator which describes a translation with respect to the given Frame. Per Default it is
         * relative to the Position of the robot base.
         * @param translation offset of the current position to the goal position
         * @param frame a Frame of Reference
         * @param maybe_orientation_generator an orientation generator for the movement. If there is no orientation generator this function
         * will generate one which does not change the orientation at all
         * @return a PoseGenerator which describes a relative offset
         */
        PoseGenerator RelativeMotion(const Position &translation, Frame frame = Frame::RobotBase,
                                     const boost::optional<OrientationGenerator> &maybe_orientation_generator = boost::none);

        /**
         * Generates a PoseGenerator which describes a movement from the given frame to the desired position.
         * @param translation the goal position
         * @param frame a Frame of Reference
         * @param maybe_orientation_generator an orientation generator for the movement. If there is no orientation generator this function
         * will generate one which does not change the orientation at all
         * @return a PoseGenerator which describes a movement to the desired position
         */
        PoseGenerator AbsoluteMotion(const Position &translation, Frame frame = Frame::RobotBase,
                                     const boost::optional<OrientationGenerator> &maybe_orientation_generator = boost::none);

        /**
         * Generates a PoseGenerator which describes a movement from the given frame to the desired pose
         * @param dest goal pose
         * @param frame a Frame of Reference
         * @return a PoseGenerator the the desired Pose
         */
        PoseGenerator MoveToPose(const Pose &dest, Frame frame = Frame::RobotBase);

        /**
         * Generates a Bezier PoseGenerator. It only cares about position so you can add an extra OrientationGenerator.
         * @note This PoseGenerator does not produce equidistant points, so the derivative wrt to time is not constant
         *  this means that it is not possible to properly plan the velocity as some part of the movements are faster than others. sry
         * @param points vector of control points
         * @param frame Reference Frame
         * @param maybe_orientation_generator  an optional orientation generator otherwise a constant orientation OrientationGenerator will be used
         * @return Bezier PoseGenerator
         */
        PoseGenerator BezierMotion(const std::vector<Position> &points, Frame frame = Frame::RobotBase,
                                   const boost::optional<OrientationGenerator> &maybe_orientation_generator = boost::none);

        /**
         * Generates a B-Spline PoseGenerator. The B-Spline only cares about position so you can add an extra OrientationGenerator
         * @note This PoseGenerator does not produce equidistant points, so the derivative wrt to time is not constant
         *  this means that it is not possible to properly plan the velocity as some part of the movements are faster than others. sry
         * @param points vector of control points
         * @param degree degree of the B-Spline polynomial
         * @param frame Reference Frame
         * @param maybe_orientation_generator an optional orientation generator otherwise a constant orientation OrientationGenerator will be used
         * @param maybe_knot_vector vector of knot positions. will assign a default one if you dont want to set your own
         * @return B-Spline PoseGenerator
         */
        PoseGenerator BSplineMotion(const std::vector<Position> &points, int degree, Frame frame = Frame::RobotBase,
                                    const boost::optional<OrientationGenerator> &maybe_orientation_generator = boost::none,
                                    const boost::optional<std::vector<int>> &maybe_knot_vector = boost::none);
    }

    /**
     * calculates the length of a PoseGenerator given a guess of the initial position
     * @param pose_generator a PoseGenerator whose length should be determined
     * @param start_pose a guess of where the pose start
     * @param steps how many samples of the trajectory should be taken (you should use an even number)
     * @return estimated length of the PoseGenerator
     */
    double calculate_length(const PoseGenerator &pose_generator, const Pose &start_pose, int steps = 10000);
}

#endif //LIBORL_POSEGENERATOR_H
