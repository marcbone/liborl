// Copyright (c) 2020 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
/*
 * Implementation of Useful functions to calculate basic geometric values
 */
#include <liborl/geometry_math.h>
#include <vector>
#include <boost/math/special_functions/factorials.hpp>

using boost::math::factorial;

double angle_between(const Eigen::Vector3d &a, const Eigen::Vector3d &b, const Eigen::Vector3d &normal) {
    return std::atan2(b.cross(a).dot(normal.normalized()), a.dot(b));
}

std::function<Eigen::Vector3d(double progress)>
generate_circle_movement(const Eigen::Vector3d &center, const Eigen::Vector3d &normal, const Eigen::Vector3d &start,
                         double rotation_angle) {


    //project center:
    const double radius = std::abs((center - start).norm());
    Eigen::Vector3d center_direction = center - start;
    // v1 = [0,1,0] x normal aka the new y direction
    auto v1 = normal.cross(center_direction);
    v1.normalize();
    Eigen::Vector3d v2 = normal.cross(v1); // the new x direction
    auto tmp_start = radius * v1;
    double start_angle = angle_between(start - center, tmp_start, normal);
    std::function<Eigen::Vector3d(double)> circle_generator = [=](double progress) -> Eigen::Vector3d {
        double desired_angle = start_angle + progress * rotation_angle;
        Eigen::Vector3d point_on_circle =
                center + radius * (v1 * std::cos(desired_angle) + v2 * std::sin(desired_angle));
        return point_on_circle;
    };
    return circle_generator;
}

std::function<Eigen::Vector3d(double progress)>
generate_circle_movement_xy_plane(const Eigen::Vector3d &center, const Eigen::Vector3d &start, double rotation_angle) {
    Eigen::Vector3d normal;
    normal << 0, 0, 1;
    Eigen::Vector3d projected_center = center;
    projected_center[2] = start[2];

    return generate_circle_movement(projected_center, normal, start, rotation_angle);
}

std::function<Eigen::Vector3d(double progress)>
generate_circle_movement_yz_plane(const Eigen::Vector3d &center, const Eigen::Vector3d &start, double rotation_angle) {
    Eigen::Vector3d normal;
    normal << 1, 0, 0;
    Eigen::Vector3d projected_center = center;
    projected_center[0] = start[0];
    return generate_circle_movement(projected_center, normal, start, rotation_angle);
}

std::function<Eigen::Vector3d(double progress)>
generate_circle_movement_zx_plane(const Eigen::Vector3d &center, const Eigen::Vector3d &start, double rotation_angle) {
    Eigen::Vector3d normal;
    normal << 0, 1, 0;
    Eigen::Vector3d projected_center = center;
    projected_center[1] = start[1];
    return generate_circle_movement(projected_center, normal, start, rotation_angle);
}


std::function<Eigen::Vector3d(double progress)> generate_bezier_curve(const std::vector<Eigen::Vector3d> &points) {
    auto C = [](int n, int i) -> double {
        return factorial<double>(n) / (factorial<double>(i) * factorial<double>(n - i));
    };
    auto J = [C](int n, int i, double t) -> double {
        return C(n, i) * std::pow(t, i) * std::pow(1 - t, n - i);
    };
    int n = points.size() - 1;
    std::function<Eigen::Vector3d(double)> bezier_generator = [=](double progress) -> Eigen::Vector3d {
//        Eigen::Vector3d sum(points.at(0));
        Eigen::Vector3d sum(0, 0, 0);
        for (int i = 0; i <= n; ++i) {
            sum += points.at(i) * J(n, i, progress);
        }
        return sum;
    };
    return bezier_generator;

}

Eigen::Vector3d
b_spline_de_boor(int k, int p, double x, const std::vector<int> &t, const std::vector<Eigen::Vector3d> &c) {
    std::vector<Eigen::Vector3d> d;
    for (int j = 0; j < p + 1; ++j) {
        d.push_back(c[j + k - p]);
    }
    for (int r = 1; r < p + 1; ++r) {
        for (int j = p; j > r - 1; --j) {
            double alpha = (x - t[j + k - p]) / (t[j + 1 + k - r] - t[j + k - p]);
            d[j] = (1.0 - alpha) * d[j - 1] + alpha * d[j];
        }
    }
    return d[p];
}


std::vector<int> create_default_knot_vector(int num_control_points, int degree) {
    int knot_vector_size = num_control_points + degree + 1;
    std::vector<int> knot_vector(knot_vector_size, 0);

    // special case to  create [0,1,2,..] vector for degree 1
    if (degree == 1) {
        for (int i = 0; i < knot_vector_size; ++i) {
            knot_vector[i] = i;
        }
        return knot_vector;
    }
    int current_knot_element = 0;
    for (int i = degree; i < knot_vector_size - degree - 1; ++i) {
        knot_vector[i] = current_knot_element++;
    }
    for (int i = knot_vector_size - degree - 1; i < knot_vector_size; ++i) {
        knot_vector[i] = current_knot_element;
    }
    return knot_vector;
}

std::function<Eigen::Vector3d(double progress)>
generate_b_spline(const std::vector<Eigen::Vector3d> &points, int degree) {
    return generate_b_spline(points, degree, create_default_knot_vector(points.size(), degree));
}


std::function<Eigen::Vector3d(double progress)>
generate_b_spline(const std::vector<Eigen::Vector3d> &points, int degree, const std::vector<int> &knot_vector) {
    if (knot_vector.size() != points.size() + degree + 1) {
        std::stringstream ss;
        ss << "The knot vector has the wrong length. It should have #points + degree of polynomial + 1 = "
           << points.size() + degree + 1 << " components." << std::endl << "But it has " << knot_vector.size()
           << " components";
        throw std::invalid_argument(ss.str());
    }
    auto b_spline_generator = [=](double progress) {
        auto p = degree;
        int k = 0;
        progress = progress * (knot_vector[knot_vector.size() - p - 1] - knot_vector[p]) + knot_vector[p];
        for (int j = p; j < (int) knot_vector.size() - 1 - p; ++j) {
            if (knot_vector[j] <= progress and progress <= knot_vector[j + 1]) {
                k = j;
                break;
            }
        }
        return b_spline_de_boor(k, p, progress, knot_vector, points);
    };
    return b_spline_generator;
}
