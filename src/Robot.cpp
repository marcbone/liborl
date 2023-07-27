// Copyright (c) 2020 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
#include <thread>
#include <liborl/liborl.h>
#include <franka/exception.h>
#include "QuinticPolynomial.hpp"
#include <liborl/Robot.h>
#include <liborl/geometry_math.h>
#include <boost/optional.hpp>
#include <liborl/Payload.h>
#include "examples_common.h"

using namespace orl;

void
Robot::relative_cart_motion(double x, double y, double z, const double max_time,
                            const boost::optional<StopCondition> &stop_condition) {
    PoseGenerator pose_generator = PoseGenerators::RelativeMotion(Position(x, y, z), Frame::RobotBase);
    apply_speed_profile(pose_generator);
    move_cartesian(pose_generator, max_time, stop_condition);

}


void Robot::cart_motion(const Pose &dest, const double max_time, const boost::optional<StopCondition> &stop_condition) {
    PoseGenerator pose_generator = PoseGenerators::MoveToPose(dest);
    apply_speed_profile(pose_generator);
    move_cartesian(pose_generator, max_time, stop_condition);
}


void Robot::close_gripper(double width, double speed, double force, double epsilon_width) {
    gripper.grasp(width, speed, force, epsilon_width, epsilon_width);
}

void Robot::open_gripper(double speed, double width) {
    gripper.move(width, speed);
}

void Robot::wait_milliseconds(int milliseconds) {
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

Pose Robot::get_current_pose() {
    return Pose(robot.readOnce().O_T_EE_c);
}

std::array<double, 7> Robot::get_current_Joints() {
    return robot.readOnce().q;
}

Robot::Robot(const std::string &robot_name) : robot(robot_name), gripper(robot_name), model(robot.loadModel()) {
    robot.setGuidingMode({true, true, true, true, true, true}, false);


    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    setDefaultBehavior();
    Payload zero_payload = Payloads::Sphere(0, 0, {0, 0, 0});
    setLoad(zero_payload);
}

void Robot::setDefaultBehavior() {
    robot.setCollisionBehavior(
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
    robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
}

franka::Model &Robot::get_franka_model() {
    return model;
}

franka::Robot &Robot::get_franka_robot() {
    return robot;
}

void
Robot::impedance_mode(double translational_stiffness, double rotational_stiffness, const Pose &desired_attractor_pose,
                      double max_time, double move_to_attractor_duration) {
    auto attractor_pose_generator = PoseGenerators::MoveToPose(desired_attractor_pose);
    apply_speed_profile(attractor_pose_generator);
    impedance_mode(translational_stiffness, rotational_stiffness, max_time, attractor_pose_generator,
                   move_to_attractor_duration);
}

void
Robot::impedance_mode(double translational_stiffness, double rotational_stiffness,
                      double max_time, const PoseGenerator &attractor_pose_generator,
                      double move_to_attractor_duration) {

    Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
    stiffness.setZero();
    stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
    stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
    damping.setZero();
    damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
                                   Eigen::MatrixXd::Identity(3, 3);
    damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
                                       Eigen::MatrixXd::Identity(3, 3);
    try {
        // connect to robot
        franka::RobotState initial_state = robot.readOnce();
        orl::Pose initial_pose(initial_state.O_T_EE);

        // set collision behavior
        robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                   {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                   {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                   {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
        // define callback for the torque control loop
        double time = 0;
        std::function<franka::Torques(const franka::RobotState &, franka::Duration)>
                impedance_control_callback = [&](const franka::RobotState &robot_state,
                                                 franka::Duration duration) -> franka::Torques {
            // get state variables
            time += duration.toSec();
            if (time == 0) {
                initial_pose.set(robot_state.O_T_EE_c);
                // Create 1D interpolation progress variable
                Eigen::Vector3d init1, fin1;
                init1 << 0, 0, 0;
                fin1 << 1, 1, 1;

            }
            const double progress = time / move_to_attractor_duration;
            PoseGeneratorInput generatorInput{progress, initial_pose, robot_state};

            orl::Pose new_pose(attractor_pose_generator(generatorInput));
            Eigen::Affine3d progress_transform(Eigen::Matrix4d::Map(new_pose.to_matrix().data()));
            Eigen::Vector3d position_d(progress_transform.translation());
            Eigen::Quaterniond orientation_d(progress_transform.linear());
            std::array<double, 7> coriolis_array = model.coriolis(robot_state);
            std::array<double, 42> jacobian_array =
                    model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
            // convert to Eigen
            Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
            Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
            Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
            Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

            Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
            Eigen::Vector3d position(transform.translation());
            Eigen::Quaterniond orientation(transform.linear());
            // compute error to desired equilibrium pose
            // position error
            Eigen::Matrix<double, 6, 1> error;
            error.head(3) << position - position_d;
            // orientation error
            // "difference" quaternion
            if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
                orientation.coeffs() << -orientation.coeffs();
            }
            // "difference" quaternion
            Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
            error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
            // Transform to base frame
            error.tail(3) << -transform.linear() * error.tail(3);
            // compute control
            Eigen::VectorXd tau_task(7), tau_d(7);
            // Spring damper system with damping ratio=1
            tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));
            tau_d << tau_task + coriolis;
            std::array<double, 7> tau_d_array{};
            Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;
            if (time > max_time) {
                return franka::MotionFinished(franka::Torques(tau_d_array));
            }
            return tau_d_array;
        };
        robot.control(impedance_control_callback, false);
    } catch (const franka::Exception &ex) {
        // print exception
        std::cout << ex.what() << std::endl;
    }
    setDefaultBehavior();
}

void Robot::impedance_mode(double translational_stiffness, double rotational_stiffness, double max_time) {
    impedance_mode(translational_stiffness, rotational_stiffness, get_current_pose(), max_time, 1);
}

void Robot::move_cartesian(PoseGenerator cartesian_pose_generator, double max_time,
                           const boost::optional<StopCondition> &stop_condition, boost::optional<double> elbow) {
    Pose initial_pose(robot.readOnce().O_T_EE_c);
    std::array<double, 2> initial_elbow;
    double time = 0.0;
    bool should_stop = false;
    try {
        robot.control(
                [=, &initial_elbow, &time, &max_time, &initial_pose, &cartesian_pose_generator, &should_stop](
                        const franka::RobotState &state,
                        franka::Duration time_step) -> franka::CartesianPose {
                    time += time_step.toSec();
                    if (time == 0) {
                        initial_pose.set(state.O_T_EE_c);
                        initial_elbow = state.elbow_c;
                    }
                    auto new_elbow = initial_elbow;
                    const double progress = time / max_time;
                    PoseGeneratorInput generatorInput{progress, initial_pose, state};
                    Pose new_pose_pose = cartesian_pose_generator(generatorInput);
                    std::array<double, 16> new_pose = new_pose_pose.to_matrix();
                    if (elbow.is_initialized()) {
                        new_elbow[0] += (elbow.value() - initial_elbow[0]) * (1 - std::cos(M_PI * progress)) / 2.0;
                    }
                    if (stop_condition.is_initialized()) {
                        should_stop = stop_condition.value()(generatorInput);
                    }
                    if (time >= max_time or should_stop) {
                        if (elbow.is_initialized()) {
                            return franka::MotionFinished({new_pose, new_elbow});
                        }
                        return franka::MotionFinished(new_pose);
                    }
                    if (elbow.is_initialized()) {
                        return {new_pose, new_elbow};
                    }
                    return new_pose;
                }, franka::ControllerMode::kCartesianImpedance, false);
    } catch (const franka::Exception& exception) {
        std::cout << exception.what() << std::endl;
        if (should_stop) {
            std::cout << "robot has stopped due to StopCondition" << std::endl;
            robot.automaticErrorRecovery();
        } else {
            throw std::move(exception);
        }
    }

}

void Robot::move_circle_xy(const Position &center, double rotation_angle, double max_time,
                           const boost::optional<OrientationGenerator> &maybe_orientation_generator,
                           const boost::optional<StopCondition> &stop_condition) {
    move_circle(center, rotation_angle, max_time, Plane::XY, maybe_orientation_generator, stop_condition);
}

void Robot::move_circle_yz(const Position &center, double rotation_angle, double max_time,
                           const boost::optional<OrientationGenerator> &maybe_orientation_generator,
                           const boost::optional<StopCondition> &stop_condition) {
    move_circle(center, rotation_angle, max_time, Plane::YZ, maybe_orientation_generator, stop_condition);
}

void Robot::move_circle_zx(const Position &center, double rotation_angle, double max_time,
                           const boost::optional<OrientationGenerator> &maybe_orientation_generator,
                           const boost::optional<StopCondition> &stop_condition) {
    move_circle(center, rotation_angle, max_time, Plane::ZX, maybe_orientation_generator, stop_condition);
}

void Robot::line_gripper_up_to_orientation(const Orientation &desired_orientation, double max_time) {
    auto orientation_gen = generate_orientation_interpolation_orientation_generator(desired_orientation.quaternion);
    auto a = generate_pose_generator(generate_constant_position_position_generator(), orientation_gen);
    move_cartesian(a, max_time);

}

void Robot::move_circle(const Position &center, double rotation_angle, double max_time, Plane plane,
                        const boost::optional<OrientationGenerator> &maybe_orientation_generator,
                        const boost::optional<StopCondition> &stop_condition) {


    PoseGenerator pose_generator = PoseGenerators::CirclePoseGenerator(center, rotation_angle, plane,
                                                                       maybe_orientation_generator);
    apply_speed_profile(pose_generator);
    move_cartesian(pose_generator, max_time, stop_condition);
}

franka::Gripper &Robot::get_franka_gripper() {
    return gripper;
}

void Robot::double_tap_robot_to_continue() {
    auto s = robot.readOnce();
    int touch_counter = false;
    bool can_be_touched_again = true;
    Eigen::Vector3d start_force;
    auto last_time_something_happened = std::chrono::system_clock::now();
    auto last_time_something_touched = std::chrono::system_clock::now();
    start_force << s.O_F_ext_hat_K[0], s.O_F_ext_hat_K[1], s.O_F_ext_hat_K[2];
    robot.read(
            [&](
                    const franka::RobotState &robot_state) {
                Eigen::Vector3d force;
                force << robot_state.O_F_ext_hat_K[0], robot_state.O_F_ext_hat_K[1], robot_state.O_F_ext_hat_K[2];
                if ((start_force - force).norm() > 2 and can_be_touched_again) {
                    touch_counter++;
                    can_be_touched_again = false;
                    last_time_something_happened = std::chrono::system_clock::now();
                    last_time_something_touched = std::chrono::system_clock::now();
                }
                if (touch_counter > 0 and (start_force - force).norm() < 1) {
                    can_be_touched_again = true;
                    last_time_something_happened = std::chrono::system_clock::now();
                }
                std::chrono::duration<double> elapse_time_touched =
                        std::chrono::system_clock::now() - last_time_something_touched;
                if (elapse_time_touched.count() > 0.5) {
                    touch_counter = 0;
                }
                std::chrono::duration<double> elapse_time_happend =
                        std::chrono::system_clock::now() - last_time_something_happened;
                if (elapse_time_happend.count() > 2) {
                    start_force = force;
                    last_time_something_happened = std::chrono::system_clock::now();

                }
                return touch_counter < 2;
            });
    wait_milliseconds(100);
}

void Robot::force_to_continue(double force, Axis axis) {

    auto s = robot.readOnce();

    Eigen::Vector3d start_force;
    start_force << s.O_F_ext_hat_K[0], s.O_F_ext_hat_K[1], s.O_F_ext_hat_K[2];
    robot.read(
            [&](
                    const franka::RobotState &robot_state) {
                Eigen::Vector3d force_v;
                force_v << robot_state.O_F_ext_hat_K[0], robot_state.O_F_ext_hat_K[1], robot_state.O_F_ext_hat_K[2];
                return std::abs(start_force[axis] - force_v[axis]) < force;
            });
}

bool Robot::force_to_continue(double force, Axis axis, double time_out) {
    auto s = robot.readOnce();

    Eigen::Vector3d start_force;
    start_force << s.O_F_ext_hat_K[0], s.O_F_ext_hat_K[1], s.O_F_ext_hat_K[2];
    int counter = 0;
    robot.read([&](const franka::RobotState &robot_state) {
        counter++;
        Eigen::Vector3d force_v;
        force_v << robot_state.O_F_ext_hat_K[0], robot_state.O_F_ext_hat_K[1], robot_state.O_F_ext_hat_K[2];
        if (counter > time_out * 1000) {
            return false;
        }
        return std::abs(start_force[axis] - force_v[axis]) < force;
    });
    return counter < time_out * 1000;
}

void Robot::torque_to_continue(double torque, Axis axis) {

    auto s = robot.readOnce();

    Eigen::Vector3d start_force;
    start_force << s.O_F_ext_hat_K[3], s.O_F_ext_hat_K[4], s.O_F_ext_hat_K[5];
    robot.read(
            [&](
                    const franka::RobotState &robot_state) {
                Eigen::Vector3d force_v;
                force_v << robot_state.O_F_ext_hat_K[3], robot_state.O_F_ext_hat_K[4], robot_state.O_F_ext_hat_K[5];
                return std::abs(start_force[axis] - force_v[axis]) < torque;
            });
}

bool Robot::torque_to_continue(double torque, Axis axis, double time_out) {

    auto s = robot.readOnce();

    Eigen::Vector3d start_force;
    start_force << s.O_F_ext_hat_K[3], s.O_F_ext_hat_K[4], s.O_F_ext_hat_K[5];
    int counter = 0;
    robot.read(
            [&](
                    const franka::RobotState &robot_state) {
                counter++;
                Eigen::Vector3d force_v;
                force_v << robot_state.O_F_ext_hat_K[3], robot_state.O_F_ext_hat_K[4], robot_state.O_F_ext_hat_K[5];
                if (counter > time_out * 1000) {
                    return false;
                }
                return std::abs(start_force[axis] - force_v[axis]) < torque;
            });
    return counter < time_out * 1000;
}

void Robot::setLoad(const Payload &load) {
    this->payload = load;
    this->robot.setLoad(payload.mass, payload.pos_wrt_flange, payload.inertia_matrix);
}

void Robot::absolute_cart_motion(double x, double y, double z, double max_time,
                                 const boost::optional<StopCondition> &stop_condition) {
    PoseGenerator pose_generator = PoseGenerators::AbsoluteMotion(Position(x, y, z), Frame::RobotBase);
    apply_speed_profile(pose_generator);
    move_cartesian(pose_generator, max_time, stop_condition);
}

void Robot::joint_motion(std::array<double, 7> q_goal, double speed_factor) {
    MotionGenerator motion_generator(speed_factor, q_goal);
    robot.control(motion_generator, franka::ControllerMode::kJointImpedance, false);
}
