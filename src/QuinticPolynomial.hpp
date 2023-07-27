// Copyright (c) 2020 Pouya Mohammadi
// Licensed under the EUPL-1.2-or-later
/*
 * File:   QuinticPolynomial.hpp
 * Author: Pouya Mohammadi
 *
 * Created on Friday June 3, 2015, 11:57
 * This is based on quintic polynomial presented in
 * Prof. Allesandro de Luca's slides of robotics 1.
 */

#ifndef QUINTICPOLYNOMIAL_HPP
#define QUINTICPOLYNOMIAL_HPP


#include <eigen3/Eigen/Core>

template<class T>
class QuinticPolynomial {
public:
    typedef Eigen::Matrix<T, Eigen::Dynamic, 1> Vector;

    QuinticPolynomial() {}

    QuinticPolynomial(double start_time, double end_time, Vector init_conf, Vector final_conf) {
        this->start_time = start_time;
        this->end_time = end_time;
        this->deltaT = end_time - start_time;
        this->q_i = init_conf;
        this->q_f = final_conf;
        this->delta_q = q_f - q_i;
        this->dof = q_i.size();
    }

    void setParams(double start_time, double end_time, Vector init_conf, Vector final_conf) {
        this->start_time = start_time;
        this->end_time = end_time;
        this->deltaT = end_time - start_time;
        this->q_i = init_conf;
        this->q_f = final_conf;
        this->delta_q = q_f - q_i;
        this->dof = q_i.size();
    }

    Vector getQ(double time) const {
        Vector ret(dof);
        if (time >= end_time)
            time = end_time;

        double tau = (time - start_time) / (deltaT);
        for (int i = 0; i < dof; ++i) {
            ret(i) = q_i(i) +
                     delta_q(i) * (6.0 * std::pow(tau, 5.0) - 15.0 * std::pow(tau, 4.0) + 10.0 * std::pow(tau, 3.0));
        }
        return ret;
    }

    void getQ(double time, Eigen::Matrix<double, 3, 1> _ret) {
        // Vector ret(dof);
        if (time >= end_time)
            time = end_time;

        double tau = (time - start_time) / (deltaT);
        for (int i = 0; i < dof; ++i) {
            _ret(i) = q_i(i) +
                      delta_q(i) * (6.0 * std::pow(tau, 5.0) - 15.0 * std::pow(tau, 4.0) + 10.0 * std::pow(tau, 3.0));
        }
    }

    Vector getQd(double time) {
        Vector ret(dof);
        if (time >= end_time)
            time = end_time;

        double tau = (time - start_time) / (deltaT);
        for (int i = 0; i < dof; ++i) {
            ret(i) = delta_q(i) * (30.0 * std::pow(tau, 4.0) - 60.0 * std::pow(tau, 3.0) + 30.0 * std::pow(tau, 2.0));
        }

        return ret;
    }

    void getQd(double time, Vector &_ret) {
        // Vector ret(dof);
        if (time >= end_time)
            time = end_time;

        double tau = (time - start_time) / (deltaT);
        for (int i = 0; i < dof; ++i) {
            _ret(i) = delta_q(i) * (30.0 * std::pow(tau, 4.0) - 60.0 * std::pow(tau, 3.0) + 30.0 * std::pow(tau, 2.0));
        }
    }

    Vector getQdd(double time) {
        Vector ret(dof);
        if (time >= end_time)
            time = end_time;

        double tau = (time - start_time) / (deltaT);
        for (int i = 0; i < dof; ++i) {
            ret(i) = delta_q(i) * (120.0 * std::pow(tau, 3.0) - 180.0 * std::pow(tau, 2.0) + 60 * tau);
        }

        return ret;
    }

    void getQdd(double time, Vector &_ret) {
        // Vector ret(dof);
        if (time >= end_time)
            time = end_time;

        double tau = (time - start_time) / (deltaT);
        for (int i = 0; i < dof; ++i) {
            _ret(i) = delta_q(i) * (120.0 * std::pow(tau, 3.0) - 180.0 * std::pow(tau, 2.0) + 60 * tau);
        }
    }

    int getDofSize() {
        return this->dof;
    }

//    void setInitialConf(Vector init);
private:
    double start_time;
    double end_time;
    double deltaT;

    int dof;

    Vector q_i;
    Vector q_f;
    Vector delta_q;
};

#endif // QUINTICPOLYNOMIAL_HPP
