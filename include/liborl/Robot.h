// Copyright (c) 2020 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
/**
 * @file Robot.h
 * @brief High-Level functions for the franka::Robot to allow fast and simple work with the robot
 * @authors Marco Boneberger
 */

#ifndef LIBORL_ROBOT_H
#define LIBORL_ROBOT_H

#include <liborl/enums.h>
#include <boost/optional.hpp>
#include <franka/robot.h>
#include <franka/gripper.h>
#include <franka/model.h>
#include <liborl/Pose.h>
#include <liborl/PoseGenerator.h>
#include <liborl/Payload.h>
#include <liborl/StopCondition.h>

namespace orl {
    /**
     * Wrapper around franka::Robot with more high_level functionality.
     *
     * Use orl::Robot::get_franka_robot() to access the low-level functionality of franka::Robot
     */
    class Robot {
    public:
        /**
         * Establishes a connection to the Franka robot and sets collision and force thresholds
         * @param robot_name name or IP of the Franka robot
         */
        explicit Robot(const std::string &robot_name);

        /**
         * gets the current End-Effector Pose
         * @return Current End-Effector Pose
         */
        Pose get_current_pose();

        /**
         * Opens the gripper
         * @param speed opening speed in m/s
         * @param width opening width in m
         */
        void open_gripper(double speed = 0.05, double width = 0.1);

        /**
         * Closes the gripper
         * @param width target width
         * @param speed closing speed
         * @param force grasping force in Newton
         * @param epsilon_width Maximum tolerated deviation from the target width
         */
        void close_gripper(double width = 0, double speed = 0.05, double force = 100, double epsilon_width = 0.1);

        /**
         * Today I don't feel like doing anything
         * @param milliseconds wait time
         */
        static void wait_milliseconds(int milliseconds);

        /**
         * gets the current joint configuration
         * @return
         */
        std::array<double, 7> get_current_Joints();

        /**
         * Relative Cartesian motion in x, y, z direction.
         * The orientation stays the same
         * \warning This command will move the robot!
         * @param x in meter
         * @param y in meter
         * @param z in meter
         * @param max_time duration of the trajectory in seconds
          * @param stop_condition a Stop condition which stops the movement
         */
        void relative_cart_motion(double x, double y, double z, double max_time = 10,
                                  const boost::optional<StopCondition> &stop_condition = boost::none);

        /**
         * Absolute Cartesian motion to x, y, z position,
         * The orientation stays the same
         * \warning This command will move the robot!
         * @param x in meter
         * @param y in meter
         * @param z in meter
         * @param max_time duration of the trajectory in seconds
         * @param stop_condition a Stop condition which stops the movement
         */
        void absolute_cart_motion(double x, double y, double z, double max_time = 10,
                                  const boost::optional<StopCondition> &stop_condition = boost::none);

        /**
         * Cartesian motion to a destination pose
         * \warning This command will move the robot!
         * @param dest destination Pose
         * @param max_time duration of the trajectory in seconds
         * @param stop_condition a Stop condition which stops the movement
         */
        void cart_motion(const Pose &dest, double max_time = 10,
                         const boost::optional<StopCondition> &stop_condition = boost::none);

        /**
         * Get Franka Robot to access low_level functionality
         * @return Reference to franka::Robot
         */
        franka::Robot &get_franka_robot();

        /**
         * Get Franka Model
         * @return Reference to franka::Model
         */
        franka::Model &get_franka_model();

        /**
         * Get Franka Gripper
         * @return Reference to franka::Gripper
         */
        franka::Gripper &get_franka_gripper();

        /**
         * Go into cartesian impedance mode.
         * The attractor point will move from the current pose the desired attractor pose
         * in the specified amount of time.
         *
         * The impedance mode will stop after max_time has passed
         * \warning This will move the robot!
         * @param translational_stiffness
         * @param rotational_stiffness
         * @param desired_attractor_pose
         * @param max_time duration of the impedance mode
         * @param move_to_attractor_duration time where the attractor point will be shifted to the goal pose
         */
        void
        impedance_mode(double translational_stiffness, double rotational_stiffness, const Pose &desired_attractor_pose,
                       double max_time = 1000000, double move_to_attractor_duration = 2);

        /**
         * Go into cartesian impedance mode
         * @param translational_stiffness
         * @param rotational_stiffness
         * @param max_time time after which the impedance mode gets aborted
         */
        void impedance_mode(double translational_stiffness, double rotational_stiffness, double max_time = 1000000);

        /**
         * * Go into cartesian impedance mode
         * The attractor point will be set by the pose Generator
         * @param translational_stiffness
         * @param rotational_stiffness
         * @param attractor_pose_generator PoseGenerator to execute
         * @param max_time duration of the impedance mode
         * @param move_to_attractor_duration execution time for the PoseGenerator
         */
        void
        impedance_mode(double translational_stiffness, double rotational_stiffness,
                       double max_time, const PoseGenerator &attractor_pose_generator,
                       double move_to_attractor_duration);

        /**
         * Moves the robot with the help of a PoseGenerator.
         * This is the most important function here and most other functions
         * are calling this function by the end of the day
         *
         *
         * @param cartesian_pose_generator a function to generate the poses
         * @param max_time time the robot has to execute the movement
         * @param stop_condition a Stop condition which stops the movement
         * @param elbow desired elbow configuration
         */
        void move_cartesian(PoseGenerator cartesian_pose_generator, double max_time,
                            const boost::optional<StopCondition> &stop_condition = boost::none,
                            boost::optional<double> elbow = boost::none);

        /**
         * lets the robot move in a circle on the x-y-plane.
         *
         * @param center center of the circle
         * @param rotation_angle how many radians to rotate
         * @param max_time how long the movement should take
         * @param maybe_orientation_generator provide an alternative orientationGenerator
         *  default is a orientation generator which does not change the orientation.
         * @param stop_condition a Stop condition which stops the movement
         */
        void move_circle_xy(const Position &center, double rotation_angle, double max_time,
                            const boost::optional<OrientationGenerator> &maybe_orientation_generator = boost::none,
                            const boost::optional<StopCondition> &stop_condition = boost::none);

        /**
         * lets the robot move in a circle on the y-z-plane.
         *
         * @param center center of the circle
         * @param rotation_angle how many radians to rotate
         * @param max_time how long the movement should take
         * @param maybe_orientation_generator provide an alternative orientationGenerator
         *  default is a orientation generator which does not change the orientation.
         *  @param stop_condition a Stop condition which stops the movement
         */
        void move_circle_yz(const Position &center, double rotation_angle, double max_time,
                            const boost::optional<OrientationGenerator> &maybe_orientation_generator = boost::none,
                            const boost::optional<StopCondition> &stop_condition = boost::none);

        /**
         * lets the robot move in a circle on the z-x-plane.
         *
         * @param center center of the circle
         * @param rotation_angle how many radians to rotate
         * @param max_time how long the movement should take
         * @param maybe_orientation_generator provide an alternative orientationGenerator
         *  default is a orientation generator which does not change the orientation.
         *  @param stop_condition a Stop condition which stops the movement
         */
        void move_circle_zx(const Position &center, double rotation_angle, double max_time,
                            const boost::optional<OrientationGenerator> &maybe_orientation_generator = boost::none,
                            const boost::optional<StopCondition> &stop_condition = boost::none);

        /**
         * moves the robot to a certain orientation
         * @param desired_orientation goal orientation
         * @param max_time time to execute the movement
         */
        void line_gripper_up_to_orientation(const Orientation &desired_orientation, double max_time);

        /**
         * waits for a human to put two times a small force on the end-effector (or a joint close to the end-effector)
         * in a short amount of time
         */
        void double_tap_robot_to_continue();

        /**
         * Set the load of the robot
         * @param load Payload object
         */
        void setLoad(const Payload &load);

        /**
         * Generates a Joint Motion to the desired goal
         * @param q_goal joint states
         * @param speed_factor value between 0 and 1 as factor of the maximum speed
         */
        void joint_motion(std::array<double, 7> q_goal, double speed_factor);

        /**
         * This function blocks until a certain amount of force is applied in a given direction
         * @param force minimum force the robot has to experience before he continues
         * @param axis an axis of the robot from which we compare the force
         */
        void force_to_continue(double force, Axis axis);

        /**
        * This function blocks until a certain amount of force is applied in a given direction or a certain time has passed
        * @param force minimum force the robot has to experience before he continues
        * @param axis an axis of the robot from which we compare the force
        * @param time_out maximum time in seconds after which the function returns
        * @return true if the forces were reached; false if the time limit has been reached
        */
        bool force_to_continue(double force, Axis axis, double time_out);

        /**
         * This function blocks until a certain amount of torque is applied on a given axis
         * @param torque minimum torque the robot has to experience before he continues
         * @param axis an axis of the robot from which we compare the torque
         */
        void torque_to_continue(double torque, Axis axis);

        /**
         * This function blocks until a certain amount of torque is applied on a given axis or a certain time has passed
         * @param torque minimum torque the robot has to experience before he continues
         * @param axis an axis of the robot from which we compare the torque
         * @param time_out maximum time in seconds after which the function returns
         * @return true if the torques were reached; false if the time limit has been reached
         */
        bool torque_to_continue(double torque, Axis axis, double time_out);

    private:
        /**
         * franka::Robot instance
         * Use get_franka_robot() to get it
         */
        franka::Robot robot;

        /**
         * franka::Gripper instance
         * Use get_franka_gripper() to get it
         */
        franka::Gripper gripper;

        /**
         * franka::Model instance
         * Use get_franka_model() to get it
         */
        franka::Model model;

        /**
         * The robots payload
         */
        Payload payload;

        /**
         * sets collision and impedance values
         */
        void setDefaultBehavior();


        /**
         * moves the robot circle on a circle given a center and a given plane of the moving
         * direction and the direction to the center.
         *
         *
         * @param center center of the circle
         * @param rotation_angle how many radians to rotate
         * @param max_time time to execute the circle movement
         * @param plane
         * @param maybe_orientation_generator
         * @param stop_condition a Stop condition which stops the movement
         */
        void move_circle(const Position &center, double rotation_angle, double max_time, Plane plane,
                         const boost::optional<OrientationGenerator> &maybe_orientation_generator,
                         const boost::optional<StopCondition> &stop_condition = boost::none);


    };
}
#endif //LIBORL_ROBOT_H
