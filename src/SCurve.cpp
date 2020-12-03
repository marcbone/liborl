// Copyright (c) 2020 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
#include <liborl/SCurve.h>

using namespace std;
using namespace SCurve;

double SCurve::deceleration_phase(const SCurveTrajectoryParameters &parameters, double t, Derivative derivative) {
    const auto &times = parameters.times;

    if (t <= times.T() - times.T_d + times.T_j2) {

        if (derivative == Derivative::Position) {
            return parameters.s.q1 - (parameters.v_lim + parameters.s.v1) * times.T_d / 2 +
                   parameters.v_lim * (t - times.T() + times.T_d) - parameters.j_max * pow(
                    t - times.T() + times.T_d, 3) / 6;
        }
        if (derivative == Derivative::Velocity) {
            return parameters.v_lim - parameters.j_max * pow(t - times.T() + times.T_d, 2) / 2;
        }
        if (derivative == Derivative::Acceleration) {
            return -parameters.j_max * (t - times.T() + times.T_d);
        }
        if (derivative == Derivative::Jerk) {
            return parameters.j_min;
        }
    }
    if (t <= times.T() - times.T_j2) {

        if (derivative == Derivative::Position) {
            return parameters.s.q1 - (parameters.v_lim + parameters.s.v1) * times.T_d / 2 +
                   parameters.v_lim * (t - times.T() + times.T_d) +
                   parameters.a_lim_d / 6 * (
                           3 * pow(t - times.T() + times.T_d, 2) - 3 * times.T_j2 * (
                                   t - times.T() + times.T_d) + pow(times.T_j2, 2));

        }
        if (derivative == Derivative::Velocity) {
            return parameters.v_lim + parameters.a_lim_d * (t - times.T() + times.T_d - times.T_j2 / 2);
        }
        if (derivative == Derivative::Acceleration) {
            return parameters.a_lim_d;
        }
        if (derivative == Derivative::Jerk) {
            return 0;
        }

    }
    if (t <= times.T()) {

        if (derivative == Derivative::Position) {
            return parameters.s.q1 - parameters.s.v1 * (times.T() - t) - parameters.j_max * pow(times.T() - t, 3) / 6;
        }
        if (derivative == Derivative::Velocity) {
            return parameters.s.v1 + parameters.j_max * pow(times.T() - t, 2) / 2;
        }
        if (derivative == Derivative::Acceleration) {
            return -parameters.j_max * (times.T() - t);
        }
        if (derivative == Derivative::Jerk) {
            return parameters.j_max;
        }
    } else {
        std::stringstream ss;
        ss << "t should be between " << times.T_a + times.T_v << " and " << times.T() << " but it is " << t;

        throw std::invalid_argument(ss.str());
    }
}

double SCurve::eval_s_curve(const SCurveTrajectoryParameters &parameters, double t, Derivative derivative) {
    const auto &times = parameters.times;
    if (t > times.T()) {
        t = times.T();
    }
    if (t <= times.T_a)
        return acceleration_phase(parameters, t, derivative);
    if (t <= times.T_a + times.T_v)
        return constant_velocity_phase(parameters, t, derivative);
    if (t <= times.T())
        return deceleration_phase(parameters, t, derivative);
    else {
        std::stringstream ss;
        ss << "t should be between 0 and " << times.T() << " but it is " << t;
        throw std::invalid_argument(ss.str());
    }
}

double SCurve::constant_velocity_phase(const SCurveTrajectoryParameters &parameters, double t, Derivative derivative) {
    const auto &times = parameters.times;
    if (derivative == Derivative::Position) {
        return parameters.s.q0 + (parameters.v_lim + parameters.s.v0) * times.T_a / 2 +
               parameters.v_lim * (t - times.T_a);
    }
    if (derivative == Derivative::Velocity) {
        return parameters.v_lim;
    }
    if (derivative == Derivative::Acceleration) {
        return 0;
    }
    if (derivative == Derivative::Jerk) {
        return 0;
    }
}

double SCurve::acceleration_phase(const SCurveTrajectoryParameters &parameters, double t, Derivative derivative) {
    const auto &times = parameters.times;
    if (t <= times.T_j1) {
        if (derivative == Derivative::Position) {
            return parameters.s.q0 + parameters.s.v0 * t + parameters.j_max * pow(t, 3) / 6;
        }
        if (derivative == Derivative::Velocity) {
            return parameters.s.v0 + parameters.j_max * pow(t, 2) / 2;
        }
        if (derivative == Derivative::Acceleration) {
            return parameters.j_max * t;
        }
        if (derivative == Derivative::Jerk) {
            return parameters.j_max;
        }
    }
    if (t <= times.T_a - times.T_j1) {

        if (derivative == Derivative::Position) {
            return parameters.s.q0 + parameters.s.v0 * t +
                   parameters.a_lim_a / 6 * (3 * pow(t, 2) - 3 * times.T_j1 * t + pow(times.T_j1, 2));
        }
        if (derivative == Derivative::Velocity) {
            return parameters.s.v0 + parameters.a_lim_a * (t - times.T_j1 / 2);
        }
        if (derivative == Derivative::Acceleration) {
            return parameters.a_lim_a;
        }
        if (derivative == Derivative::Jerk) {
            return 0;
        }
    }
    if (t <= times.T_a) {

        if (derivative == Derivative::Position) {
            return parameters.s.q0 + (parameters.v_lim + parameters.s.v0) * times.T_a / 2 -
                   parameters.v_lim * (times.T_a - t) - parameters.j_min * pow(
                    times.T_a - t, 3) / 6;
        }
        if (derivative == Derivative::Velocity) {
            return parameters.v_lim + parameters.j_min * pow(times.T_a - t, 2) / 2;
        }
        if (derivative == Derivative::Acceleration) {
            return -parameters.j_min * (times.T_a - t);
        }
        if (derivative == Derivative::Jerk) {
            return parameters.j_min;
        }
    } else {
        std::stringstream ss;
        ss << "t should be between " << 0 << " and " << times.T_a << " but it is " << t;
        throw std::invalid_argument(ss.str());
    }
}

SCurveTrajectoryParameters SCurve::generate_trajectory_parameters(const SCurveTimings &times,
                                                                  const SCurveInitializationParameters &initialization) {
    double a_lim_a = initialization.constraints.max_jerk * times.T_j1;
    double a_lim_d = -initialization.constraints.max_jerk * times.T_j2;
    double v_lim = initialization.startConditions.v0 + (times.T_a - times.T_j1) * a_lim_a;

    return SCurveTrajectoryParameters{times, initialization.constraints.max_jerk, -initialization.constraints.max_jerk,
                                      a_lim_a, a_lim_d, v_lim,
                                      {initialization.startConditions.q0, initialization.startConditions.q1,
                                       initialization.startConditions.v0,
                                       initialization.startConditions.v1}};
}

std::pair<SCurveTrajectoryParameters, std::function<double(double)>>
SCurve::s_curve_generator(const SCurveInitializationParameters &init_params, Derivative derivative) {
    auto times = init_params.calc_times();
    auto params = generate_trajectory_parameters(times, init_params);
    auto s_curve = [=](double t) {
        return eval_s_curve(params, t, derivative);
    };
    return {params, s_curve};

}

SCurveTimings &
SCurveInitializationParameters::handle_negative_acceleration_time(SCurveTimings &times,
                                                                  const SCurveInitializationParameters &new_initialization) const {
    if (times.T_a < 0) {
        times.T_j1 = 0;
        times.T_a = 0;
        times.T_d = 2 * startConditions.h() / (startConditions.v0 + startConditions.v1);
        times.T_j2 = (new_initialization.constraints.max_jerk * startConditions.h() -
                      sqrt(new_initialization.constraints.max_jerk *
                           (new_initialization.constraints.max_jerk * pow(startConditions.h(), 2) +
                            pow(startConditions.v0 + startConditions.v1, 2) * (
                                    startConditions.v1 - startConditions.v0)))) / (
                             new_initialization.constraints.max_jerk * (startConditions.v1 + startConditions.v0));
    }
    if (times.T_d < 0) {
        times.T_j2 = 0;
        times.T_d = 0;
        times.T_a = 2 * startConditions.h() / (startConditions.v0 + startConditions.v1);
        times.T_j2 = (new_initialization.constraints.max_jerk * startConditions.h() -
                      sqrt(new_initialization.constraints.max_jerk *
                           (new_initialization.constraints.max_jerk * pow(startConditions.h(), 2) -
                            pow(startConditions.v0 + startConditions.v1, 2) * (
                                    startConditions.v1 - startConditions.v0)))) / (
                             new_initialization.constraints.max_jerk * (startConditions.v1 + startConditions.v0));
    }
    return times;
}

SCurveTimings SCurveInitializationParameters::calc_times_case_1() const {
    SCurveTimings times;
    SCurveInitializationParameters new_input = *this;
    if (is_a_max_not_reached()) {
        times.T_j1 = sqrt((new_input.constraints.max_velocity - startConditions.v0) / new_input.constraints.max_jerk);
        times.T_a = 2 * times.T_j1;
    } else {
        times.T_j1 = new_input.constraints.max_acceleration / new_input.constraints.max_jerk;
        times.T_a = times.T_j1 +
                    (new_input.constraints.max_velocity - startConditions.v0) / new_input.constraints.max_acceleration;
    }

    if (is_a_min_not_reached()) {
        times.T_j2 = sqrt((new_input.constraints.max_velocity - startConditions.v1) / new_input.constraints.max_jerk);
        times.T_d = 2 * times.T_j2;
    } else {
        times.T_j2 = new_input.constraints.max_acceleration / new_input.constraints.max_jerk;
        times.T_d = times.T_j2 +
                    (new_input.constraints.max_velocity - startConditions.v1) / new_input.constraints.max_acceleration;
    }

    times.T_v = startConditions.h() / new_input.constraints.max_velocity -
                times.T_a / 2 * (1 + startConditions.v0 / new_input.constraints.max_velocity) -
                times.T_d / 2 * (1 + startConditions.v1 / new_input.constraints.max_velocity);
    if (times.T_v <= 0) {
        return calc_times_case_2();
    }
    if (times.is_max_acceleration_not_reached()) {
        new_input.constraints.max_acceleration *= 0.99;
        if (new_input.constraints.max_acceleration > 0.1) {
            return new_input.calc_times_case_2();
        }
        new_input.constraints.max_acceleration = 0;
    }
    times = handle_negative_acceleration_time(times, new_input);

    return times;
}

SCurveTimings SCurveInitializationParameters::calc_times_case_2() const {
    SCurveTimings times;
    SCurveInitializationParameters new_input = *this;
    times.T_j1 = new_input.constraints.max_acceleration / new_input.constraints.max_jerk;
    times.T_j2 = new_input.constraints.max_acceleration / new_input.constraints.max_jerk;
    double delta = pow(new_input.constraints.max_acceleration, 4) / pow(new_input.constraints.max_jerk, 2) +
                   2 * (pow(startConditions.v0, 2) + pow(startConditions.v1, 2)) +
                   new_input.constraints.max_acceleration * (4 * startConditions.h() -
                                                             2 * new_input.constraints.max_acceleration /
                                                             new_input.constraints.max_jerk *
                                                             (startConditions.v0 + startConditions.v1));
    times.T_a =
            (pow(new_input.constraints.max_acceleration, 2) / new_input.constraints.max_jerk - 2 * startConditions.v0 +
             sqrt(delta)) / (2 * new_input.constraints.max_acceleration);
    times.T_d =
            (pow(new_input.constraints.max_acceleration, 2) / new_input.constraints.max_jerk - 2 * startConditions.v1 +
             sqrt(delta)) / (2 * new_input.constraints.max_acceleration);
    times.T_v = 0;
    if (times.is_max_acceleration_not_reached()) {
        new_input.constraints.max_acceleration *= 0.99;
        if (new_input.constraints.max_acceleration > 0.1) {
            return new_input.calc_times_case_2();
        }
        new_input.constraints.max_acceleration = 0;
    }
    times = handle_negative_acceleration_time(times, new_input);

    return times;
}

bool SCurveInitializationParameters::is_a_max_not_reached() const {
    return (constraints.max_velocity - startConditions.v0) * constraints.max_jerk <
           pow(constraints.max_acceleration, 2);
}

bool SCurveInitializationParameters::is_a_min_not_reached() const {
    return (constraints.max_velocity - startConditions.v1) * constraints.max_jerk <
           pow(constraints.max_acceleration, 2);
}

bool SCurveInitializationParameters::is_trajectory_feasible() const {
    double T_j_star = fmin(sqrt(abs(startConditions.v1 - startConditions.v0) / constraints.max_jerk),
                           constraints.max_acceleration / constraints.max_jerk);
    if (T_j_star == constraints.max_acceleration / constraints.max_jerk) {
        return startConditions.h() > 0.5 * (startConditions.v1 + startConditions.v0) * (T_j_star +
                                                                                        abs(startConditions.v1 -
                                                                                            startConditions.v0) /
                                                                                        constraints.max_acceleration);
    }
    if (T_j_star < constraints.max_acceleration / constraints.max_jerk) {
        return startConditions.h() > T_j_star * (startConditions.v0 + startConditions.v1);
    }
    return false;
}

SCurveTimings SCurveInitializationParameters::calc_times() const {
    return calc_times_case_1();
}

double SCurveStartConditions::h() const {
    return q1 - q0;
}

bool SCurveTimings::is_max_acceleration_not_reached() const {
    return T_a < 2 * T_j1 or T_d < 2 * T_j2;
}

double SCurveTimings::T() const {
    return T_a + T_v + T_d;
}
