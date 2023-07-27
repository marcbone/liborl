// Copyright (c) 2020 Marco Boneberger
// Licensed under the EUPL-1.2-or-later

#include <gtest/gtest.h>
#include <liborl/liborl.h>
#include <franka/robot_state.h>

using namespace orl;
TEST(PoseGeneratorTest, LinearInterpolation) {
    orl::Pose start_pose;
    orl::Pose end_pose;
    auto pose_generator = orl::generate_pose_interpolation_pose_generator(end_pose);
    orl::apply_speed_profile(pose_generator, orl::SpeedProfiles::LinearProfile());
    auto pose_generator_quintic = orl::generate_pose_interpolation_pose_generator(end_pose);
    franka::RobotState state;
    state.O_T_EE_c = start_pose.to_matrix();
    for (int i = 0; i < 100; i++) {
        double progress = i / 100.;
        orl::PoseGeneratorInput input{progress, start_pose, state};
    }
}

TEST(PoseGeneratorTest, ConstantPositionTest) {
    Position start(1, 0, 0);

    auto pose_generator = generate_pose_generator(generate_constant_position_position_generator(),
                                                  generate_constant_orientation_orientation_generator());

    for (int i = 1; i < 100; i++) {
        std::cout << std::fixed << std::setprecision(4);
        double progress = i / 100.;
        Eigen::Quaterniond quat;
        PoseGeneratorInput input{progress, Pose(start, quat), franka::RobotState()};
        Position generator_position(pose_generator(input).getPosition());
        Eigen::Quaterniond generator_quat(pose_generator(input).quaternion());
        Position desired_position;
        desired_position << 1, 0, 0;
        ASSERT_NEAR(generator_quat.x(), quat.x(), 0.00001);
        ASSERT_NEAR(generator_quat.y(), quat.y(), 0.00001);
        ASSERT_NEAR(generator_quat.z(), quat.z(), 0.00001);
        ASSERT_NEAR((generator_position - desired_position).norm(), 0, 0.00001);
    }
}




TEST(PoseGeneratorTest, UnitCircleXYTest) {
    Position start(1, 0, 0);
    Position circle_center(0, 0, 0);
    auto pose_generator = PoseGenerators::CirclePoseGenerator(circle_center, 2 * M_PI, Plane::XY);
    for (int i = 1; i < 100; i++) {
        std::cout << std::fixed << std::setprecision(4);
        double progress = i / 100.;
        PoseGeneratorInput input{progress, Pose(start, Eigen::Quaterniond()), franka::RobotState()};
        Position generator_position(pose_generator(input).getPosition());
        Position desired_position;
        desired_position << std::cos(progress * 2 * M_PI), std::sin(progress * 2 * M_PI), 0;
        ASSERT_NEAR((generator_position - desired_position).norm(), 0, 0.00001);
    }
}

TEST(PoseGeneratorTest, UnitCircleYZTest) {
    Position start(0, 1, 0);
    Position circle_center(0, 0, 0);
    auto pose_generator = PoseGenerators::CirclePoseGenerator(circle_center, 2 * M_PI, Plane::YZ);
    for (int i = 1; i < 100; i++) {
        std::cout << std::fixed << std::setprecision(4);
        double progress = i / 100.;
        PoseGeneratorInput input{progress, Pose(start, Eigen::Quaterniond()), franka::RobotState()};
        Position generator_position(pose_generator(input).getPosition());
        Position desired_position;
        desired_position << 0, std::cos(progress * 2 * M_PI), std::sin(progress * 2 * M_PI);
        ASSERT_NEAR((generator_position - desired_position).norm(), 0, 0.00001);
    }
}

TEST(PoseGeneratorTest, UnitCircleZXTest) {
    Position start(0, 0, 1);
    Position circle_center(0, 0, 0);
    auto pose_generator = PoseGenerators::CirclePoseGenerator(circle_center, 2 * M_PI, Plane::ZX);
    for (int i = 1; i < 100; i++) {
        std::cout << std::fixed << std::setprecision(4);
        double progress = i / 100.;
        PoseGeneratorInput input{progress, Pose(start, Eigen::Quaterniond()), franka::RobotState()};
        Position generator_position(pose_generator(input).getPosition());
        Position desired_position;
        desired_position << std::sin(progress * 2 * M_PI), 0, std::cos(progress * 2 * M_PI);
        ASSERT_NEAR((generator_position - desired_position).norm(), 0, 0.00001);
    }
}

TEST(PoseGeneratorTest, RelativeMotionTest) {
    Pose start(Position(1, 0, 0), Eigen::Quaterniond());
    Position offset(0, 0, 1);
    auto pose_generator = PoseGenerators::RelativeMotion(offset, Frame::RobotBase);
    for (int i = 1; i < 100; i++) {
        std::cout << std::fixed << std::setprecision(4);
        double progress = i / 100.;
        PoseGeneratorInput input{progress, start, franka::RobotState()};
        Position generator_position(pose_generator(input).getPosition());
        Position desired_position;
        desired_position << 1, 0, progress;
        ASSERT_EQ(generator_position, desired_position);
    }
}

TEST(PoseGeneratorTest, AbsoluteMotionTest) {
    Pose start(Position(1, 0, 0), Eigen::Quaterniond());
    Position offset(0, 0, 1);
    auto pose_generator = PoseGenerators::AbsoluteMotion(offset);
    for (int i = 1; i < 100; i++) {
        std::cout << std::fixed << std::setprecision(4);
        double progress = i / 100.;
        PoseGeneratorInput input{progress, start, franka::RobotState()};
        Position generator_position(pose_generator(input).getPosition());
        Position desired_position;
        desired_position << 1 - progress, 0, progress;
        ASSERT_EQ(generator_position, desired_position);
    }
}

TEST(PoseGeneratorTest, MovePoseTest) {
    Pose start(Position(1, 0, 0), Eigen::Quaterniond());
    Pose goal(Position(0, 0, 1), Eigen::Quaterniond());
    auto pose_generator_abs = PoseGenerators::AbsoluteMotion(goal.getPosition());
    auto pose_generator_move = PoseGenerators::MoveToPose(goal);
    for (int i = 1; i < 100; i++) {
        std::cout << std::fixed << std::setprecision(4);
        double progress = i / 100.;
        PoseGeneratorInput input{progress, start, franka::RobotState()};
        Position generator_position(pose_generator_move(input).getPosition());
        Position desired_position(pose_generator_abs(input).getPosition());
        ASSERT_EQ(generator_position, desired_position);
    }
}

TEST(PoseGeneratorTest, AppliedPoseGeneratorsTest) {
    Pose start(Position(1, 0, 0), Eigen::Quaterniond());
    start.set_RPY(0, M_PI_2, M_PI);
    Position offset(0, 0, 1);
    PoseGenerator pose_generator_z = PoseGenerators::RelativeMotion(offset, Frame::RobotBase);
    PoseGenerator pose_generator_y = PoseGenerators::RelativeMotion(Position(0, 1, 0), Frame::UnitFrame);
    auto pose_generator = pose_generator_y * pose_generator_z;
    for (int i = 1; i < 100; i++) {
        std::cout << std::fixed << std::setprecision(4);
        double progress = i / 100.;
        PoseGeneratorInput input{progress, start, franka::RobotState()};
        Position generator_position(pose_generator(input).getPosition());
        Position desired_position;
        desired_position << 1, progress, progress;
        ASSERT_EQ(generator_position, desired_position);
    }
}

TEST(PoseGeneratorTest, RelativeTCPPoseGeneratorTest) {
    Pose start(Position(1, 0, 0), Eigen::Quaterniond());
    start.set_RPY(0, 0, M_PI_2);
    Position offset(1, 0, 0);
    PoseGenerator pose_generator = PoseGenerators::RelativeMotion(offset, Frame::RobotTCP);
    for (int i = 1; i < 100; i++) {
        std::cout << std::fixed << std::setprecision(4);
        double progress = i / 100.;
        PoseGeneratorInput input{progress, start, franka::RobotState()};
        Position generator_position(pose_generator(input).getPosition());
        Position desired_position;
        desired_position << 1, progress, 0;
        ASSERT_NEAR((generator_position - desired_position).norm(), 0, 0.00001);
    }
}

TEST(SpeedProfileTest, CosineTest) {

    Pose start(Position(0, 0, 0), Eigen::Quaterniond());
    Position offset(1, 0, 0);
    PoseGenerator pose_generator = PoseGenerators::RelativeMotion(offset, Frame::UnitFrame);
    apply_speed_profile(pose_generator, SpeedProfiles::CosineProfile());
    for (int i = 1; i < 100; i++) {
        std::cout << std::fixed << std::setprecision(4);
        double progress = i / 100.;
        PoseGeneratorInput input{progress, start, franka::RobotState()};
        Position generator_position(pose_generator(input).getPosition());
        Position desired_position;
        desired_position << (1 - std::cos(M_PI * progress)) / 2, 0, 0;
        ASSERT_NEAR((generator_position - desired_position).norm(), 0, 0.00001);
    }
}


TEST(PoseGeneratorTest, BezierTest) {
    Pose start(Position(1, 0, 0), Eigen::Quaterniond());
    Position offset(0, 0, 1);
    std::vector<Position> d;
    d.push_back(offset);
    auto pose_generator = PoseGenerators::BezierMotion(d);
    for (int i = 1; i < 100; i++) {
        std::cout << std::fixed << std::setprecision(4);
        double progress = i / 100.;
        PoseGeneratorInput input{progress, start, franka::RobotState()};
        Position generator_position(pose_generator(input).getPosition());
        Position desired_position;
        desired_position << 1 - progress, 0, progress;
        ASSERT_NEAR((generator_position - desired_position).norm(), 0, 0.00001);
    }
}

TEST(GeometryTest, BezierTest) {
    Pose start(Position(1, 0, 0), Eigen::Quaterniond());
    Position offset(1, 0, 1);
    std::vector<Position> d;
    d.push_back(start.getPosition());
    d.push_back(offset);
    auto pose_generator = generate_bezier_curve(d);
    for (int i = 1; i < 100; i++) {
        std::cout << std::fixed << std::setprecision(4);
        double progress = i / 100.;
        Position generator_position(pose_generator(progress));
        Position desired_position;
        desired_position << 1, 0, progress;
        ASSERT_EQ(generator_position, desired_position);
    }
}

TEST(GeometryTest, BSplineTest) {
    const std::vector<double> desired_x{0.0000e+00, 4.0000e-04, 1.6000e-03, 3.6000e-03, 6.4000e-03, 1.0000e-02,
                                        1.4400e-02, 1.9600e-02, 2.5600e-02, 3.2400e-02, 4.0000e-02, 4.8400e-02,
                                        5.7600e-02, 6.7600e-02, 7.8400e-02, 9.0000e-02, 1.0240e-01, 1.1560e-01,
                                        1.2960e-01, 1.4440e-01, 1.6000e-01, 1.7640e-01, 1.9360e-01, 2.1160e-01,
                                        2.3040e-01, 2.5000e-01, 2.7040e-01, 2.9160e-01, 3.1360e-01, 3.3640e-01,
                                        3.6000e-01, 3.8440e-01, 4.0960e-01, 4.3560e-01, 4.6240e-01, 4.9000e-01,
                                        5.1840e-01, 5.4760e-01, 5.7760e-01, 6.0840e-01, 6.4000e-01, 6.7240e-01,
                                        7.0560e-01, 7.3960e-01, 7.7440e-01, 8.1000e-01, 8.4640e-01, 8.8360e-01,
                                        9.2160e-01, 9.6040e-01, 1.0000e+00, 1.0396e+00, 1.0784e+00, 1.1164e+00,
                                        1.1536e+00, 1.1900e+00, 1.2256e+00, 1.2604e+00, 1.2944e+00, 1.3276e+00,
                                        1.3600e+00, 1.3916e+00, 1.4224e+00, 1.4524e+00, 1.4816e+00, 1.5100e+00,
                                        1.5376e+00, 1.5644e+00, 1.5904e+00, 1.6156e+00, 1.6400e+00, 1.6636e+00,
                                        1.6864e+00, 1.7084e+00, 1.7296e+00, 1.7500e+00, 1.7696e+00, 1.7884e+00,
                                        1.8064e+00, 1.8236e+00, 1.8400e+00, 1.8556e+00, 1.8704e+00, 1.8844e+00,
                                        1.8976e+00, 1.9100e+00, 1.9216e+00, 1.9324e+00, 1.9424e+00, 1.9516e+00,
                                        1.9600e+00, 1.9676e+00, 1.9744e+00, 1.9804e+00, 1.9856e+00, 1.9900e+00,
                                        1.9936e+00, 1.9964e+00, 1.9984e+00, 1.9996e+00, 2.0000e+00};
    const std::vector<double> desired_y{0., 0.0396, 0.0784, 0.1164, 0.1536, 0.19, 0.2256, 0.2604, 0.2944, 0.3276, 0.36,
                                        0.3916, 0.4224, 0.4524, 0.4816, 0.51, 0.5376, 0.5644, 0.5904, 0.6156, 0.64,
                                        0.6636, 0.6864, 0.7084, 0.7296, 0.75, 0.7696, 0.7884, 0.8064, 0.8236, 0.84,
                                        0.8556, 0.8704, 0.8844, 0.8976, 0.91, 0.9216, 0.9324, 0.9424, 0.9516, 0.96,
                                        0.9676, 0.9744, 0.9804, 0.9856, 0.99, 0.9936, 0.9964, 0.9984, 0.9996, 1.,
                                        0.9996, 0.9984, 0.9964, 0.9936, 0.99, 0.9856, 0.9804, 0.9744, 0.9676, 0.96,
                                        0.9516, 0.9424, 0.9324, 0.9216, 0.91, 0.8976, 0.8844, 0.8704, 0.8556, 0.84,
                                        0.8236, 0.8064, 0.7884, 0.7696, 0.75, 0.7296, 0.7084, 0.6864, 0.6636, 0.64,
                                        0.6156, 0.5904, 0.5644, 0.5376, 0.51, 0.4816, 0.4524, 0.4224, 0.3916, 0.36,
                                        0.3276, 0.2944, 0.2604, 0.2256, 0.19, 0.1536, 0.1164, 0.0784, 0.0396, 0.};
    ASSERT_EQ(desired_x.size(), desired_y.size());
    std::vector<Position> points{{0, 0, 0},
                                 {0, 1, 0},
                                 {2, 1, 0},
                                 {2, 0, 0}};
    std::vector<int> knot_vector{0, 0, 0, 2, 4, 4, 4};
    auto pose_generator = generate_b_spline(points, 2, knot_vector);
    auto pose_generator_2 = generate_b_spline(points, 2);
    for (int i = 0; i < 100; i++) {

        std::cout << std::fixed << std::setprecision(4);
        double progress = i / 100.;
        Position generator_position(pose_generator(progress));
        Position generator_position_2(pose_generator_2(progress));
        Position desired_position;
        desired_position << desired_x.at(i), desired_y.at(i), 0;
        ASSERT_NEAR((generator_position - desired_position).norm(), 0, 0.00001);
        ASSERT_NEAR((generator_position - generator_position_2).norm(), 0, 0.00001);
    }
}

TEST(GeometryTest, BSplineDegree1Test) {

    std::vector<Position> points{{0, 0, 0},
                                 {0, 1, 0},
    };
    auto pose_generator = generate_b_spline(points, 1);

    for (int i = 0; i <= 100; i++) {
        std::cout << std::fixed << std::setprecision(4);
        double progress = i / 100.;
        Position generator_position(pose_generator(progress));

        Position desired_position;
        desired_position << 0, progress, 0;;
        ASSERT_NEAR((generator_position - desired_position).norm(), 0, 0.0001);
    }
}

TEST(GeometryTest, BSplineMotionTest) {
    const std::vector<double> desired_x{0.0000e+00, 4.0000e-04, 1.6000e-03, 3.6000e-03, 6.4000e-03, 1.0000e-02,
                                        1.4400e-02, 1.9600e-02, 2.5600e-02, 3.2400e-02, 4.0000e-02, 4.8400e-02,
                                        5.7600e-02, 6.7600e-02, 7.8400e-02, 9.0000e-02, 1.0240e-01, 1.1560e-01,
                                        1.2960e-01, 1.4440e-01, 1.6000e-01, 1.7640e-01, 1.9360e-01, 2.1160e-01,
                                        2.3040e-01, 2.5000e-01, 2.7040e-01, 2.9160e-01, 3.1360e-01, 3.3640e-01,
                                        3.6000e-01, 3.8440e-01, 4.0960e-01, 4.3560e-01, 4.6240e-01, 4.9000e-01,
                                        5.1840e-01, 5.4760e-01, 5.7760e-01, 6.0840e-01, 6.4000e-01, 6.7240e-01,
                                        7.0560e-01, 7.3960e-01, 7.7440e-01, 8.1000e-01, 8.4640e-01, 8.8360e-01,
                                        9.2160e-01, 9.6040e-01, 1.0000e+00, 1.0396e+00, 1.0784e+00, 1.1164e+00,
                                        1.1536e+00, 1.1900e+00, 1.2256e+00, 1.2604e+00, 1.2944e+00, 1.3276e+00,
                                        1.3600e+00, 1.3916e+00, 1.4224e+00, 1.4524e+00, 1.4816e+00, 1.5100e+00,
                                        1.5376e+00, 1.5644e+00, 1.5904e+00, 1.6156e+00, 1.6400e+00, 1.6636e+00,
                                        1.6864e+00, 1.7084e+00, 1.7296e+00, 1.7500e+00, 1.7696e+00, 1.7884e+00,
                                        1.8064e+00, 1.8236e+00, 1.8400e+00, 1.8556e+00, 1.8704e+00, 1.8844e+00,
                                        1.8976e+00, 1.9100e+00, 1.9216e+00, 1.9324e+00, 1.9424e+00, 1.9516e+00,
                                        1.9600e+00, 1.9676e+00, 1.9744e+00, 1.9804e+00, 1.9856e+00, 1.9900e+00,
                                        1.9936e+00, 1.9964e+00, 1.9984e+00, 1.9996e+00, 2.0000e+00};
    const std::vector<double> desired_y{0., 0.0396, 0.0784, 0.1164, 0.1536, 0.19, 0.2256, 0.2604, 0.2944, 0.3276, 0.36,
                                        0.3916, 0.4224, 0.4524, 0.4816, 0.51, 0.5376, 0.5644, 0.5904, 0.6156, 0.64,
                                        0.6636, 0.6864, 0.7084, 0.7296, 0.75, 0.7696, 0.7884, 0.8064, 0.8236, 0.84,
                                        0.8556, 0.8704, 0.8844, 0.8976, 0.91, 0.9216, 0.9324, 0.9424, 0.9516, 0.96,
                                        0.9676, 0.9744, 0.9804, 0.9856, 0.99, 0.9936, 0.9964, 0.9984, 0.9996, 1.,
                                        0.9996, 0.9984, 0.9964, 0.9936, 0.99, 0.9856, 0.9804, 0.9744, 0.9676, 0.96,
                                        0.9516, 0.9424, 0.9324, 0.9216, 0.91, 0.8976, 0.8844, 0.8704, 0.8556, 0.84,
                                        0.8236, 0.8064, 0.7884, 0.7696, 0.75, 0.7296, 0.7084, 0.6864, 0.6636, 0.64,
                                        0.6156, 0.5904, 0.5644, 0.5376, 0.51, 0.4816, 0.4524, 0.4224, 0.3916, 0.36,
                                        0.3276, 0.2944, 0.2604, 0.2256, 0.19, 0.1536, 0.1164, 0.0784, 0.0396, 0.};
    Pose start(Position(0, 0, 0), Eigen::Quaterniond());
    ASSERT_EQ(desired_x.size(), desired_y.size());
    std::vector<Position> points{{0, 1, 0},
                                 {2, 1, 0},
                                 {2, 0, 0}};
    std::vector<int> knot_vector{0, 0, 0, 2, 4, 4, 4};
    auto pose_generator = PoseGenerators::BSplineMotion(points, 2);
    for (int i = 0; i <= 100; i++) {
        std::cout << std::fixed << std::setprecision(4);
        double progress = i / 100.;
        PoseGeneratorInput input{progress, start, franka::RobotState()};
        Position generator_position(pose_generator(input).getPosition());
        Position desired_position;
        desired_position << desired_x.at(i), desired_y.at(i), 0;
        ASSERT_NEAR((generator_position - desired_position).norm(), 0, 0.00001);
    }
}

TEST(PoseGeneratorTest, LengthTest) {
    Pose start(Position(1, 0, 0), Eigen::Quaterniond());
    Position offset(0, 0, 1);
    auto pose_generator = PoseGenerators::RelativeMotion(offset, Frame::RobotBase);
    double length = calculate_length(pose_generator, start);
    ASSERT_NEAR(length, 1, 0.00001);
    pose_generator = PoseGenerators::CirclePoseGenerator({0, 0, 0}, 2 * M_PI, Plane::XY);
    length = calculate_length(pose_generator, start);
    ASSERT_NEAR(length, 2 * M_PI, 0.00001);

}

TEST(GeometryTest, SCurveTest3_9) {
    SCurve::SCurveInitializationParameters input{{30, 10, 5},
                                                 {0,  10, 1, 0}};
    auto times = input.calc_times();
    ASSERT_NEAR(times.T_a, 0.7333, 0.001);
    ASSERT_NEAR(times.T_v, 1.1433, 0.001);
    ASSERT_NEAR(times.T_d, 0.8333, 0.001);
    ASSERT_NEAR(times.T_j1, 0.333, 0.001);
    ASSERT_NEAR(times.T_j2, 0.333, 0.001);

}

TEST(GeometryTest, SCurveTest3_10) {
    SCurve::SCurveInitializationParameters input{{30, 10, 10},
                                                 {0,  10, 1, 0}};
    auto times = input.calc_times();
    ASSERT_NEAR(times.T_a, 1.0747, 0.001);
    ASSERT_NEAR(times.T_v, 0, 0.001);
    ASSERT_NEAR(times.T_d, 1.1747, 0.001);
    ASSERT_NEAR(times.T_j1, 0.333, 0.001);
    ASSERT_NEAR(times.T_j2, 0.333, 0.001);
}

TEST(GeometryTest, SCurveTest3_11) {
    SCurve::SCurveInitializationParameters input{{30, 10, 10},
                                                 {0,  10, 7, 0}};
    auto times = input.calc_times();
    ASSERT_NEAR(times.T_a, 0.4666, 0.001);
    ASSERT_NEAR(times.T_v, 0, 0.001);
    ASSERT_NEAR(times.T_d, 1.4718, 0.001);
    ASSERT_NEAR(times.T_j1, 0.2312, 0.001);
    ASSERT_NEAR(times.T_j2, 0.2321, 0.001);
}

TEST(GeometryTest, SCurveTest3_12) {
    SCurve::SCurveInitializationParameters input{{30, 10, 10},
                                                 {0,  10, 7.5, 0}};
    auto times = input.calc_times();
    ASSERT_NEAR(times.T_a, 0, 0.001);
    ASSERT_NEAR(times.T_v, 0, 0.001);
    ASSERT_NEAR(times.T_d, 2.6667, 0.001);
    ASSERT_NEAR(times.T_j1, 0, 0.001);
    ASSERT_NEAR(times.T_j2, 0.0973, 0.001);
}

TEST(GeometryTest, SCurveSimpleTest) {
    std::vector<double> desired_positions{0.0, 0.0272, 0.055, 0.08399, 0.11477, 0.14794, 0.18409, 0.22383, 0.26775,
                                          0.31644, 0.37051, 0.43055, 0.49716, 0.57089, 0.65197, 0.7404, 0.83597,
                                          0.93816, 1.04635, 1.15994, 1.27836, 1.40098, 1.52723, 1.65649, 1.78819,
                                          1.9217, 2.05645, 2.19183, 2.32733, 2.46283, 2.59833, 2.73383, 2.86933,
                                          3.00483, 3.14033, 3.27583, 3.41133, 3.54683, 3.68233, 3.81783, 3.95333,
                                          4.08883, 4.22433, 4.35983, 4.49533, 4.63083, 4.76633, 4.90183, 5.03733,
                                          5.17283, 5.30833, 5.44383, 5.57933, 5.71483, 5.85033, 5.98583, 6.12133,
                                          6.25683, 6.39233, 6.52783, 6.66333, 6.79883, 6.93433, 7.06983, 7.20533,
                                          7.34083, 7.47633, 7.61183, 7.74733, 7.88283, 8.01829, 8.1533, 8.28726,
                                          8.41958, 8.54967, 8.67691, 8.80072, 8.92051, 9.03566, 9.14559, 9.2497,
                                          9.34739, 9.43807, 9.52143, 9.59744, 9.6661, 9.72743, 9.78141, 9.82804,
                                          9.86755, 9.90049, 9.92746, 9.94905, 9.96587, 9.97851, 9.98756, 9.99363,
                                          9.99731, 9.9992, 9.9999, 10.0};
    std::vector<double> desired_velocities{1.0, 1.01102, 1.04406, 1.09915, 1.17626, 1.2754, 1.39658, 1.53979, 1.70503,
                                           1.89231, 2.10161, 2.33295, 2.58633, 2.85633, 3.12733, 3.3977, 3.6524,
                                           3.88507, 4.0957, 4.2843, 4.45087, 4.59541, 4.71792, 4.81839, 4.89683,
                                           4.95324, 4.98762, 4.99996, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0,
                                           5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0,
                                           5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0,
                                           5.0, 5.0, 4.9938, 4.96625, 4.91667, 4.84506, 4.75142, 4.63574, 4.49803,
                                           4.33829, 4.15652, 3.95271, 3.72687, 3.479, 3.21133, 2.94033, 2.66933,
                                           2.39833, 2.12733, 1.85633, 1.58633, 1.33295, 1.10161, 0.89231, 0.70503,
                                           0.53979, 0.39658, 0.2754, 0.17626, 0.09915, 0.04406, 0.01102, 0.0};
    std::vector<double> desired_accelerations{0.0, 0.813, 1.626, 2.439, 3.252, 4.065, 4.878, 5.691, 6.504, 7.317, 8.13,
                                              8.943, 9.756, 10.0, 10.0, 9.805, 8.992, 8.179, 7.366, 6.553, 5.74, 4.927,
                                              4.114, 3.301, 2.488, 1.675, 0.862, 0.049, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.61, -1.423, -2.236, -3.049, -3.862,
                                              -4.675, -5.488, -6.301, -7.114, -7.927, -8.74, -9.553, -10.0, -10.0,
                                              -10.0, -10.0, -10.0, -10.0, -9.756, -8.943, -8.13, -7.317, -6.504, -5.691,
                                              -4.878, -4.065, -3.252, -2.439, -1.626, -0.813, -0.0};
    std::vector<double> desired_jerks{30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 0.0,
                                      0.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0,
                                      -30.0, -30.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -30.0, -30.0,
                                      -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, 0.0, 0.0,
                                      0.0, 0.0, 0.0, 0.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0,
                                      30.0, 30.0, 30.0};;
    std::vector<std::vector<double>> desired{desired_positions, desired_velocities, desired_accelerations,
                                             desired_jerks};
    SCurve::SCurveInitializationParameters input{{30, 10, 5},
                                                 {0,  10, 1, 0}};

    for (int j = 0; j < 4; ++j) {
        auto gen = SCurve::s_curve_generator(input, SCurve::Derivative(j));
        const auto &params = gen.first;
        const auto s_curve = gen.second;
        for (int i = 0; i < 101; ++i) {
            double progress = i / 100. * params.times.T();
            s_curve(progress);
            ASSERT_NEAR(desired[j][i], s_curve(progress), 0.0001);
        }
    }

}

TEST(GeometryTest, SCurveComplexTest) {
    std::vector<double> desired_positions{0., 0.98838207, 1.92947655, 2.81867756, 3.6559851, 4.44139917, 5.17491978,
                                          5.85654691, 6.48628058, 7.06412078, 7.59006752, 8.06412078, 8.48628058,
                                          8.85654691, 9.17491978, 9.44139917, 9.6559851, 9.81867756, 9.92947655,
                                          9.98838207, 10.};
    std::vector<double> desired_velocities{7.5, 7.25280908, 6.86360807, 6.47440706, 6.08520605, 5.69600504,
                                           5.30680403, 4.91760303, 4.52840202, 4.13920101, 3.75, 3.36079899,
                                           2.97159798, 2.58239697, 2.19319597, 1.80399496, 1.41479395, 1.02559294,
                                           0.63639193, 0.24719092, 0.};
    std::vector<double> desired_accelerations{0., -2.91900756, -2.91900756, -2.91900756, -2.91900756, -2.91900756,
                                              -2.91900756, -2.91900756, -2.91900756, -2.91900756, -2.91900756,
                                              -2.91900756, -2.91900756, -2.91900756, -2.91900756, -2.91900756,
                                              -2.91900756, -2.91900756, -2.91900756, -2.91900756, -0.};
    std::vector<double> desired_jerks{30., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                                      0., 30.};
    std::vector<std::vector<double>> desired{desired_positions, desired_velocities, desired_accelerations,
                                             desired_jerks};
    SCurve::SCurveInitializationParameters input{{30, 10, 10},
                                                 {0,  10, 7.5, 0}};
    for (int j = 0; j < 4; ++j) {
        auto gen = SCurve::s_curve_generator(input, SCurve::Derivative(j));
        const auto &params = gen.first;
        const auto s_curve = gen.second;
        for (int i = 0; i < 21; ++i) {
            double progress = i / 20. * params.times.T();
            ASSERT_NEAR(desired[j][i], s_curve(progress), 0.0001);
        }
    }

}

TEST(PoseGeneratorTest, SCurve) {
    std::vector<double> desired_positions{0.0, 0.00011, 0.00091, 0.00307, 0.00728, 0.01422, 0.02456, 0.03901, 0.05823,
                                          0.08291, 0.11373, 0.15137, 0.19652, 0.24964, 0.3108, 0.37998, 0.45719,
                                          0.54242, 0.63568, 0.7367, 0.84481, 0.95936, 1.07964, 1.20498, 1.33469, 1.4681,
                                          1.60452, 1.74326, 1.88365, 2.02501, 2.16667, 2.30833, 2.45, 2.59167, 2.73333,
                                          2.875, 3.01667, 3.15833, 3.3, 3.44167, 3.58333, 3.725, 3.86667, 4.00833, 4.15,
                                          4.29167, 4.43333, 4.575, 4.71667, 4.85833, 5.0, 5.14167, 5.28333, 5.425,
                                          5.56667, 5.70833, 5.85, 5.99167, 6.13333, 6.275, 6.41667, 6.55833, 6.7,
                                          6.84167, 6.98333, 7.125, 7.26667, 7.40833, 7.55, 7.69167, 7.83333, 7.97499,
                                          8.11635, 8.25674, 8.39548, 8.5319, 8.66531, 8.79502, 8.92036, 9.04064,
                                          9.15519, 9.2633, 9.36432, 9.45758, 9.54281, 9.62002, 9.6892, 9.75036, 9.80348,
                                          9.84863, 9.88627, 9.91709, 9.94177, 9.96099, 9.97543, 9.98578, 9.99272,
                                          9.99693, 9.99909, 9.99989, 10.0};
    Pose start(Position(1, 0, 1), Eigen::Quaterniond());
    auto pose_generator = orl::PoseGenerators::RelativeMotion({0, 0, 10});
    double trajectory_duration;
    franka::RobotState state;
    apply_speed_profile(pose_generator, SpeedProfiles::SCurveProfile(pose_generator, start.getPosition(), 30, 10, 5,
                                                                     trajectory_duration));
    for (int i = 0; i <= 100; i++) {
        std::cout << std::fixed << std::setprecision(4);
        double progress = i / 100.;
        orl::PoseGeneratorInput input{progress, start, state};
        Position generator_position(pose_generator(input).getPosition());
        Position desired_position;
        desired_position << 1, 0, 1 + desired_positions[i];
        ASSERT_NEAR((generator_position - desired_position).norm(), 0, 0.00001);
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
