// Copyright (c) 2020 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
/*
 * Implementation of Generator functions
 */

#include <liborl/PoseGenerator.h>
#include <liborl/SpeedProfile.h>
#include <liborl/geometry_math.h>

using namespace orl;

PoseGenerator
orl::generate_pose_generator(const PositionGenerator &position_generator,
                             const OrientationGenerator &orientation_generator,
                             const boost::optional<SpeedProfile> &speed_profile) {
    PoseGenerator pose_generator = [=](const PoseGeneratorInput &input) -> orl::Pose {
        orl::Pose out(position_generator(input), orientation_generator(input).normalized());
        return out;
    };
    SpeedProfile concrete_speed_profile = speed_profile.value_or(SpeedProfiles::QuinticPolynomialProfile());
    apply_speed_profile(pose_generator, concrete_speed_profile);
    return pose_generator;
}

OrientationGenerator
orl::generate_constant_orientation_orientation_generator(const boost::optional<Eigen::Quaterniond> &orientation) {
    if (orientation.is_initialized()) {
        return [=](const PoseGeneratorInput &input) -> Eigen::Quaterniond {
            return orientation.value();
        };
    }
    OrientationGenerator constant_orientation_generator = [=](const PoseGeneratorInput &input) -> Eigen::Quaterniond {
        return input.initial_pose.quaternion();
    };
    return constant_orientation_generator;
}


OrientationGenerator orl::generate_angle_axis_orientation_generator(const Eigen::AngleAxisd &angle_axis) {
    OrientationGenerator constant_orientation_generator = [=](const PoseGeneratorInput &input) -> Eigen::Quaterniond {
        Eigen::Quaterniond desired_orientation(angle_axis);
        desired_orientation *= input.initial_pose.quaternion();
        return generate_orientation_interpolation_orientation_generator(desired_orientation)(input);
    };
    return constant_orientation_generator;
}

PositionGenerator orl::generate_constant_position_position_generator(const boost::optional<Position> &position) {
    if (position.is_initialized()) {
        return [=](const PoseGeneratorInput &input) -> Eigen::Vector3d {
            return position.value();
        };
    }
    PositionGenerator constant_position_generator = [=](const PoseGeneratorInput &input) -> Eigen::Vector3d {
        return input.initial_pose.translation();
    };
    return constant_position_generator;
}


PoseGenerator orl::generate_pose_interpolation_pose_generator(const orl::Pose &end_pose) {
    orl::Pose d = end_pose;
    PoseGenerator pose_generator = [=](const PoseGeneratorInput &input) -> orl::Pose {
        return input.initial_pose.interpolate(d, input.progress);


    };
    return pose_generator;
}

OrientationGenerator
orl::generate_orientation_interpolation_orientation_generator(const Eigen::Quaterniond &end_orientation) {
    return [=](const PoseGeneratorInput &input) -> Eigen::Quaterniond {
        return input.initial_pose.quaternion().slerp(input.progress, end_orientation);
    };
}

PoseGenerator orl::operator*(const PoseGenerator &lhs, const PoseGenerator &rhs) {
    PoseGenerator pose_generator = [=](const PoseGeneratorInput &input) -> orl::Pose {
        return lhs(input) * rhs(input);
    };
    return pose_generator;
}

double orl::calculate_length(const PoseGenerator &pose_generator, const Pose &start_pose, int steps) {
    std::vector<Position> positions;
    positions.reserve(steps + 1);
    for (int i = 0; i <= steps; i++) {
        double progress = i / (double) steps;
        PoseGeneratorInput input{progress, start_pose, franka::RobotState()};
        positions.push_back(pose_generator(input).getPosition());
    }
    double sum = 0.0;
    for (int i = 0; i < int(positions.size()) - 1; ++i) {
        sum += (positions[i] - positions[i + 1]).norm();
    }
    return sum;
}


PoseGenerator PoseGenerators::CirclePoseGenerator(const Position &center, double rotation_angle, const Plane &plane,
                                                  const boost::optional<OrientationGenerator> &maybe_orientation_generator) {
    Eigen::Vector3d normal;
    switch (plane) {
        case Plane::XY:
            normal << 0, 0, 1;
            break;
        case Plane::YZ:
            normal << 1, 0, 0;
            break;
        case Plane::ZX:
        default:
            normal << 0, 1, 0;
    }
    PositionGenerator position_generator = [=](const PoseGeneratorInput &input) {
        auto circle_generator_initializer = [=](Plane plane) {

            switch (plane) {
                case Plane::XY:
                    return generate_circle_movement_xy_plane(center, input.initial_pose.translation(),
                                                             rotation_angle);
                case Plane::YZ:
                    return generate_circle_movement_yz_plane(center, input.initial_pose.translation(),
                                                             rotation_angle);
                case Plane::ZX:
                default:
                    return generate_circle_movement_zx_plane(center, input.initial_pose.translation(),
                                                             rotation_angle);
            }
        };

        /*
         * \todo make this more efficient by not recalculating this every time.
         * Make sure that it still works if two threads use this function at the same time
         */
        auto circle_generator = circle_generator_initializer(plane);
        return circle_generator(input.progress);
    };

    Eigen::AngleAxisd orient_to_center(rotation_angle, normal);
    auto orientation_generator = maybe_orientation_generator.value_or(
            generate_angle_axis_orientation_generator(orient_to_center));
    auto pose_generator = generate_pose_generator(position_generator, orientation_generator,
                                                  SpeedProfiles::LinearProfile());
    return pose_generator;
}

PoseGenerator PoseGenerators::RelativeMotion(const Position &translation, Frame frame,
                                             const boost::optional<OrientationGenerator> &maybe_orientation_generator) {

    PositionGenerator position_generator = [=](const PoseGeneratorInput &input) -> orl::Position {
        orl::Pose goal;
        orl::Pose start;
        switch (frame) {
            case Frame::RobotBase:
                goal = start = input.initial_pose;
                goal.add_position(translation.x(), translation.y(), translation.z());
                break;
            case Frame::UnitFrame:
                goal = start = {Position(0, 0, 0), Orientation(0, 0, 0).quaternion};
                goal.add_position(translation.x(), translation.y(), translation.z());
                break;
            case Frame::RobotTCP:
                start = input.initial_pose;
                goal = start * Pose(translation, Eigen::Quaterniond());
        }

        return start.getPosition() + (-start.getPosition() + goal.getPosition()) * input.progress;
    };
    auto orientation_generator = maybe_orientation_generator.value_or(
            generate_constant_orientation_orientation_generator());
    if (frame == Frame::UnitFrame) {
        orientation_generator = maybe_orientation_generator.value_or(
                generate_constant_orientation_orientation_generator(Orientation(0, 0, 0).quaternion));
    }
    auto pose_generator = generate_pose_generator(position_generator, orientation_generator,
                                                  SpeedProfiles::LinearProfile());
    return pose_generator;
}

PoseGenerator PoseGenerators::AbsoluteMotion(const Position &translation, Frame frame,
                                             const boost::optional<OrientationGenerator> &maybe_orientation_generator) {
    PositionGenerator position_generator = [=](const PoseGeneratorInput &input) -> orl::Position {
        orl::Position goal;
        orl::Position start;
        switch (frame) {
            case Frame::RobotTCP:
                throw std::invalid_argument("RobotTCP in absolute motion does not make any sense");
            case Frame::RobotBase:
                goal = start = input.initial_pose.getPosition();
                goal = translation;
                break;
            case Frame::UnitFrame:
                std::cerr
                        << "Absolute motion in Unit Frame equals Relative Motion in Unit Frame. Use Relative Motion the next time"
                        << std::endl;
                goal = start = {0, 0, 0};
                goal = (translation);
                break;

        }
        return start + (-start + goal) * input.progress;
    };
    auto orientation_generator = maybe_orientation_generator.value_or(
            generate_constant_orientation_orientation_generator());
    auto pose_generator = generate_pose_generator(position_generator, orientation_generator,
                                                  SpeedProfiles::LinearProfile());
    return pose_generator;
}

PoseGenerator PoseGenerators::BezierMotion(const std::vector<Position> &points, Frame frame,
                                           const boost::optional<OrientationGenerator> &maybe_orientation_generator) {
    PoseGenerator pose_generator = [=](const PoseGeneratorInput &input) -> orl::Pose {

        orl::Pose start;
        if (frame == Frame::UnitFrame) {
            start = Pose(Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond());
        } else {
            start = input.initial_pose;
        }
        std::vector<Position> points_with_start{start.getPosition()};
        orl::Pose goal = start;
        goal.setPosition(points.at(points.size() - 1));
        for (const auto &point : points) {
            points_with_start.push_back(point);
        }
        auto position_generator = generate_bezier_curve(points_with_start);
        auto orientation_generator = maybe_orientation_generator.value_or(
                generate_constant_orientation_orientation_generator());
        return Pose(position_generator(input.progress), orientation_generator(input));
    };
    return pose_generator;
}

PoseGenerator PoseGenerators::MoveToPose(const Pose &dest, Frame frame) {
    PoseGenerator pose_generator;
    switch (frame) {
        case Frame::RobotTCP:
            throw std::invalid_argument("RobotTCP in MoveToPose does not make any sense");
        case Frame::RobotBase:
            pose_generator = generate_pose_interpolation_pose_generator(dest);
            break;
        case Frame::UnitFrame:
            pose_generator = [=](const PoseGeneratorInput &input) -> orl::Pose {
                return Pose(Position(0, 0, 0), Eigen::Quaterniond()).interpolate(dest, input.progress);
            };
            break;

    }
    return pose_generator;
}

PoseGenerator PoseGenerators::BSplineMotion(const std::vector<Position> &points, int degree, Frame frame,
                                            const boost::optional<OrientationGenerator> &maybe_orientation_generator,
                                            const boost::optional<std::vector<int>> &maybe_knot_vector) {
    if (points.size() < 2) {
        throw std::invalid_argument("To few points for B-Spline. Need at least 2");
    }
    // add one to the points size, to make up for the initial position of the robot which is not known yet
    std::vector<int> knot_vector = maybe_knot_vector.value_or(create_default_knot_vector(points.size() + 1, degree));
    PoseGenerator pose_generator = [=](const PoseGeneratorInput &input) -> orl::Pose {

        orl::Pose start;
        if (frame == Frame::UnitFrame) {
            start = Pose(Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond());
        } else {
            start = input.initial_pose;
        }
        std::vector<Position> points_with_start{start.getPosition()};
        orl::Pose goal = start;
        goal.setPosition(points.at(points.size() - 1));
        for (const auto &point : points) {
            points_with_start.push_back(point);
        }
        auto position_generator = generate_b_spline(points_with_start, degree, knot_vector);
        auto orientation_generator = maybe_orientation_generator.value_or(
                generate_constant_orientation_orientation_generator());
        return Pose(position_generator(input.progress), orientation_generator(input));
    };
    return pose_generator;
}
