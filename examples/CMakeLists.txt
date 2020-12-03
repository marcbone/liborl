cmake_minimum_required(VERSION 3.5)
project(examples)

set(CMAKE_CXX_STANDARD 14)
find_package(Threads)

include_directories(
        # include
        SYSTEM
        PRIVATE
        .
)

add_executable(cartesian_motion_example force_stop_example.cpp)

target_link_libraries(cartesian_motion_example
        orl
        )

add_executable(gripper_control gripper_control.cpp)

target_link_libraries(gripper_control
        orl
        )
add_executable(check_joint_states check_joint_states.cpp)

target_link_libraries(check_joint_states
        orl
        )

add_executable(circle_example circle_example.cpp)

target_link_libraries(circle_example
        orl
        )
add_executable(pose_generators_example pose_generators_example.cpp)

target_link_libraries(pose_generators_example
        orl
        )

add_executable(bezier_example bezier_example.cpp)

target_link_libraries(bezier_example
        orl
        )

add_executable(b_spline_example b_spline_example.cpp)

target_link_libraries(b_spline_example
        orl
        )
add_executable(s_curve_example s_curve_example.cpp)

target_link_libraries(s_curve_example
        orl
        )
add_executable(impedance_example impedance_example.cpp)

target_link_libraries(impedance_example
        orl
        )