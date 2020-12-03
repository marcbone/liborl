// Copyright (c) 2020 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
/**
 * @file SpeedProfile.h
 * @brief defines SpeedProfiles and offers a set of SpeedProfiles that can be used
 * @authors Marco Boneberger
 */

#ifndef LIBORL_SPEEDPROFILE_H
#define LIBORL_SPEEDPROFILE_H

#include <liborl/PoseGenerator.h>
#include <liborl/SCurve.h>

namespace orl {

    namespace SpeedProfiles {

        /**
         * Generates a SpeedProfile which uses a polynomial of degree 5. This is the Default SpeedProfile
         * @return a QuinticPolynomial SpeedProfile
         */
        SpeedProfile QuinticPolynomialProfile();

        /**
         * Generates a SpeedProfile which does not alter the PoseGenerator. Use it when you do not want your PoseGenerator to
         * be implicitly converted to a Quintic Polynomial
         * @return a Linear SpeedProfile
         */
        SpeedProfile LinearProfile();

        /**
         * Generates a SpeedProfile which uses 1 - cos(x)
         * @return a Cosine SpeedProfile
         */
        SpeedProfile CosineProfile();

        /**
         * Generates an S-Curve Profile if you already know the trajectory parameters.
         * @param params Parameters of the S-Curve
         * @return S-Curve Speed Profile
         */
        SpeedProfile SCurveProfile(const SCurve::SCurveTrajectoryParameters &params);

        /**
         * Generates an S-Curve Speed Profile
         *
         * @warning If it is not possible to fulfill your trajectory the acceleration and velocity will be decreased
         * @param[in] pg a the Pose Generator for which the profile should be calculated
         * @param[in] initial_position  an initial guess of the start position
         * @param[in] jerk maximum allowed jerk for your profile
         * @param[in] acceleration maximum allowed acceleration for your profile
         * @param[in] velocity maximum allowed velocity for your profile
         * @param[out] trajectory_duration duration of the trajectory
         * @return S-Curve Speed Profile
         */
        SpeedProfile
        SCurveProfile(const PoseGenerator &pg, const Position &initial_position, double jerk, double acceleration,
                      double velocity, double &trajectory_duration);

        /**
         * Generates an S-Curve Speed Profile
         * @warning If it is not possible to fulfill your trajectory the acceleration and velocity will be decreased.
         * They are writen in your params variable
         * @param[in] pg a the Pose Generator for which the profile should be calculated
         * @param[in] initial_position  an initial guess of the start position
         * @param[in] jerk maximum allowed jerk for your profile
         * @param[in] acceleration maximum allowed acceleration for your profile
         * @param[in] velocity maximum allowed velocity for your profile
         * @param[out] params the calculated trajectory parameters
         * @return
         */
        SpeedProfile
        SCurveProfile(const PoseGenerator &pg, const Position &initial_position, double jerk, double acceleration,
                      double velocity, SCurve::SCurveTrajectoryParameters &params);
    }

    /**
     * applies a SpeedProfile to a PoseGenerator
     * @param pose_generator
     * @param speed_profile
     */
    void apply_speed_profile(PoseGenerator &pose_generator,
                             const SpeedProfile &speed_profile = SpeedProfiles::QuinticPolynomialProfile());

}

#endif //LIBORL_SPEEDPROFILE_H
