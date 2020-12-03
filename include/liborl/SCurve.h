// Copyright (c) 2020 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
/**
 * @file SCurve.h
 * @brief contains necessary structs and methods to calculate S-Curves
 * @authors Marco Boneberger
 */

#ifndef LIBORL_SCURVE_H
#define LIBORL_SCURVE_H

#include <cmath>
#include <functional>
#include <sstream>


namespace SCurve {

/**
 * Struct which contains the desired limits for jerk, acceleration and velocity in SI units
 */
    struct SCurveConstraints {
        double max_jerk = 0;
        double max_acceleration = 0;
        double max_velocity = 0;
    };

    /// Enum which is used to select whether you want to calculate
    /// Position, Velocity, Acceleration or Jerk of your S-Curve
    enum Derivative {
        Position = 0, Velocity = 1, Acceleration = 2, Jerk = 3
    };

    /**
     * Struct which contains the time intervals of the S-Curve
     */
    struct SCurveTimings {
        double T_j1 = 0;///< time-interval in which the jerk is constant (j max or j min ) during the acceleration phase
        double T_j2 = 0; ///< time-interval in which the jerk is constant (j max or j min ) during the deceleration phase
        double T_a = 0;///< Acceleration period
        double T_v = 0;///< constant velocity period
        double T_d = 0;///< deceleration period

        /**
         *  total duration of the trajectory ( = T a + T v + T d )
         * @return duration in seconds
         */
        double T() const;

        /**
         * determines if the maximum acceleration is reaches
         * @return
         */
        bool is_max_acceleration_not_reached() const;
    };

    /// struct in which the start and end conditions are stored
    struct SCurveStartConditions {
        double q0 = 0; ///< start position
        double q1 = 1; ///< end position
        double v0 = 0; ///< start velocity
        double v1 = 0; ///< end velocity

        /**
         * displacement form start q0 to end q1
         * @return displacement in meter
         */
        double h() const;
    };

    /**
     * Parameters of the SCurve. This is the set of variable which completely defines the trajectory
     */
    struct SCurveTrajectoryParameters {
        SCurveTimings times;///< S-Curve time intervals
        double j_max = 0; ///< max jerk
        double j_min = 0;///< min jerk
        double a_lim_a = 0;///< maximum acceleration in the acceleration phase
        double a_lim_d = 0;///< maximum acceleration in the deceleration phase

        double v_lim = 0;///< start position
        SCurveStartConditions s;///< start position

    };

    /**
     * Input parameters which are necessary to generate a S-Curve. This is not the same as  the SCurveTrajectoryParameters but you can
     * calculate them with the initialization parameters
     */
    struct SCurveInitializationParameters {
        SCurveConstraints constraints; ///< constraints for the S-Curve
        SCurveStartConditions startConditions; ///< start conditions
    public:
        /**
         * calculates the time intervals for the parameters
         * @return
         */
        SCurveTimings calc_times() const;

        /**
         * checks if a trajectory is feasible
         * @return 
         */
        bool is_trajectory_feasible() const;

    private:
        /**
         * checks if the maximum acceleration can not be reached
         * @return 
         */
        bool is_a_max_not_reached() const;

        /**
         * checks if the minimum acceleration can not be reached
         * @return 
         */
        bool is_a_min_not_reached() const;


        /**
         * calculates the times if the maximum velocity can not be reached
         * @return 
         */
        SCurveTimings calc_times_case_2() const;

        /**
         * calculates the times if the maximum velocity can be reached
         * @return 
         */
        SCurveTimings calc_times_case_1() const;

        /**
         * This function checks if the acceleration or deceleration phase are negative and fixes them
         * @param times current time intervals
         * @param new_initialization set of initialization parameters to try it again
         * @return 
         */
        SCurveTimings &handle_negative_acceleration_time(SCurveTimings &times,
                                                         const SCurveInitializationParameters &new_initialization) const;
    };

    /**
     * generates the trajectory parameters
     * @param times S-Curve intervals
     * @param initialization trajectory initialization parameters
     * @return a fully defined set of S-Curve Parameters
     */
    SCurveTrajectoryParameters
    generate_trajectory_parameters(const SCurveTimings &times, const SCurveInitializationParameters &initialization);

    /**
     * Calculates values of the S-Curve during the Acceleration Phase
     * @param parameters trajectory parameters
     * @param t time in seconds. this is the function input
     * @param derivative What derivative you are interested in. Default is position
     * @return value of the S-Curve at time t
     */
    double acceleration_phase(const SCurveTrajectoryParameters &parameters, double t,
                              Derivative derivative = Derivative::Position);

    /**
     * Calculates values of the S-Curve during the Constant Velocity Phase
     * @param parameters trajectory parameters
     * @param t time in seconds. this is the function input
     * @param derivative What derivative you are interested in. Default is position
     * @return value of the S-Curve at time t
     */
    double
    constant_velocity_phase(const SCurveTrajectoryParameters &parameters, double t,
                            Derivative derivative = Derivative::Position);

    /**
     * Calculates values of the S-Curve during the Deceleration Phase
     * @param parameters trajectory parameters
     * @param t time in seconds. this is the function input
     * @param derivative What derivative you are interested in. Default is position
     * @return value of the S-Curve at time t
     */
    double deceleration_phase(const SCurveTrajectoryParameters &parameters, double t,
                              Derivative derivative = Derivative::Position);

    /**
     * Evaluates the S-Curve at a given time
     * @param parameters trajectory parameters
     * @param t time in seconds. this is the function input
     * @param derivative What derivative you are interested in. Default is position
     * @return value of the S-Curve at time t
     */
    double
    eval_s_curve(const SCurveTrajectoryParameters &parameters, double t, Derivative derivative = Derivative::Position);

    /**
     * This function generates a function which takes time as input and returns the value of the S-Curve at the given time
     * It also returns the Parameters of the S-Curve
     * @param init_params initialization parameters for the S-Curve
     * @param derivative What derivative you are interested in. Default is position
     * @return SCurveTrajectoryParameters and SCurve function
     */
    std::pair<SCurveTrajectoryParameters, std::function<double(double)>>
    s_curve_generator(const SCurveInitializationParameters &init_params, Derivative derivative = Derivative::Position);
}

#endif //LIBORL_SCURVE_H
