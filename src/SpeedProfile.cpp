// Copyright (c) 2020 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
#include <liborl/SpeedProfile.h>
#include "QuinticPolynomial.hpp"
#include <liborl/SCurve.h>

using namespace orl;

void orl::apply_speed_profile(PoseGenerator &pose_generator, const SpeedProfile &speed_profile) {
    pose_generator = [pose_generator, speed_profile](const PoseGeneratorInput &input) {
        PoseGeneratorInput new_input = input;
        new_input.progress = speed_profile(input.progress);
        return pose_generator(new_input);
    };
}

SpeedProfile SpeedProfiles::QuinticPolynomialProfile() {
    Eigen::Vector3d init;
    init << 0, 0, 0;
    Eigen::Vector3d fin;
    fin << 1, 1, 1;
    QuinticPolynomial<double> qp(0, 1, init, fin);
    qp.setParams(0, 1, init, fin);
    return [qp](double progress) {
        return qp.getQ(progress).x();
    };
}

SpeedProfile SpeedProfiles::LinearProfile() {
    return [](double progress) {
        return progress;
    };
}

SpeedProfile SpeedProfiles::CosineProfile() {
    return [](double progress) {
        return (1 - std::cos(M_PI * progress)) / 2.0;
    };
}

SpeedProfile SpeedProfiles::SCurveProfile(const SCurve::SCurveTrajectoryParameters &params) {
    const double trajectory_duration = params.times.T();
    return [=](double progress) {
        return SCurve::eval_s_curve(params, progress * trajectory_duration, SCurve::Derivative::Position);
    };

}

SpeedProfile SpeedProfiles::SCurveProfile(const PoseGenerator &pg, const Position &initial_position, double jerk,
                                          double acceleration, double velocity, double &trajectory_duration) {
    SCurve::SCurveTrajectoryParameters params;
    double trajectory_length = calculate_length(pg, Pose(initial_position, Eigen::Quaterniond()));
    SCurve::SCurveInitializationParameters input{{jerk, acceleration,      velocity},
                                                 {0,    trajectory_length, 0, 0}};
    auto times = input.calc_times();
    params = SCurve::generate_trajectory_parameters(times, input);
    trajectory_duration = params.times.T();
    return [=](double progress) {
        return SCurve::eval_s_curve(params, progress * trajectory_duration, SCurve::Derivative::Position) /
               trajectory_length;
    };

}

SpeedProfile SpeedProfiles::SCurveProfile(const PoseGenerator &pg, const Position &initial_position, double jerk,
                                          double acceleration, double velocity,
                                          SCurve::SCurveTrajectoryParameters &params) {
    double trajectory_length = calculate_length(pg, Pose(initial_position, Eigen::Quaterniond()));
    SCurve::SCurveInitializationParameters input{{jerk, acceleration,      velocity},
                                                 {0,    trajectory_length, 0, 0}};
    auto times = input.calc_times();
    params = SCurve::generate_trajectory_parameters(times, input);
    double trajectory_duration = params.times.T();
    return [=](double progress) {
        return SCurve::eval_s_curve(params, progress * trajectory_duration, SCurve::Derivative::Position) /
               trajectory_length;
    };
}
