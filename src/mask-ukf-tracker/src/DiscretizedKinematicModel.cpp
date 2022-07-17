/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <DiscretizedKinematicModel.h>

#include <Eigen/Dense>

using namespace Eigen;
using namespace bfl;


DiscretizedKinematicModel::DiscretizedKinematicModel
(
    const double sigma_x,  const double sigma_y,  const double sigma_z,
    const double sigma_yaw, const double sigma_pitch, const double sigma_roll,
    const double period, const bool estimate_period
    )
{
    estimate_period_ = estimate_period;

    sigma_position_.resize(3);
    sigma_position_ << sigma_x, sigma_y, sigma_z;

    sigma_orientation_.resize(3);
    sigma_orientation_ << sigma_yaw, sigma_pitch, sigma_roll;

    F_.resize(12, 12);
    F_ = MatrixXd::Zero(12, 12);

    Q_.resize(12, 12);
    Q_ = MatrixXd::Zero(12, 12);

    // Evaluate F and Q matrices using a default
    // sampling time to be updated online
    evaluateStateTransitionMatrix(period);
    evaluateNoiseCovarianceMatrix(period);
}


DiscretizedKinematicModel::~DiscretizedKinematicModel()
{ }


void DiscretizedKinematicModel::evaluateStateTransitionMatrix(const double T)
{
    // Compute the state transition matrix
    F_.block<3, 3>(0, 0) = Matrix3d::Identity();
    F_.block<3, 3>(0, 3) = T * Matrix3d::Identity();
    F_.block<3, 3>(3, 3) = Matrix3d::Identity();
    F_.block<3, 3>(6, 6) = Matrix3d::Identity();
    F_.block<3, 3>(9, 6) = T * Matrix3d::Identity();
    F_.block<3, 3>(9, 9) = Matrix3d::Identity();
}


void DiscretizedKinematicModel::evaluateNoiseCovarianceMatrix(const double T)
{
    // Compose noise covariance matrix for the linear acceleration part
    MatrixXd Q_pos(6, 6);
    Q_pos.block<3, 3>(0, 0) = sigma_position_.asDiagonal() * (std::pow(T, 3.0) / 3.0);
    Q_pos.block<3, 3>(0, 3) = sigma_position_.asDiagonal() * (std::pow(T, 2.0) / 2.0);
    Q_pos.block<3, 3>(3, 0) = sigma_position_.asDiagonal() * (std::pow(T, 2.0) / 2.0);
    Q_pos.block<3, 3>(3, 3) = sigma_position_.asDiagonal() * T;

    // Compose noise covariance matrix for the euler angle rates part
    MatrixXd Q_ang(6, 6);
    Q_ang.block<3, 3>(0, 0) = sigma_orientation_.asDiagonal() * T;
    Q_ang.block<3, 3>(0, 3) = sigma_orientation_.asDiagonal() * (std::pow(T, 2.0) / 2.0);
    Q_ang.block<3, 3>(3, 0) = sigma_orientation_.asDiagonal() * (std::pow(T, 2.0) / 2.0);
    Q_ang.block<3, 3>(3, 3) = sigma_orientation_.asDiagonal() * (std::pow(T, 3.0) / 3.0);

    Q_.block<6, 6>(0, 0) = Q_pos;
    Q_.block<6, 6>(6, 6) = Q_ang;
}


VectorDescription DiscretizedKinematicModel::getInputDescription()
{
    // 9 linear components (x, y, z, x_dot, y_dot, z_dot, yaw_dot, pitch_dot, roll_dot)
    // 3 angular components (yaw, pitch, roll)
    // 12 noise components
    return VectorDescription(9, 3, 12, VectorDescription::CircularType::Euler);
}


VectorDescription DiscretizedKinematicModel::getStateDescription()
{
    // 9 linear components (x, y, z, x_dot, y_dot, z_dot, yaw_dot, pitch_dot, roll_dot)
    // 3 angular components (yaw, pitch, roll)
    // 0 noise components
    return VectorDescription(9, 3, 0, VectorDescription::CircularType::Euler);
}


Eigen::MatrixXd DiscretizedKinematicModel::getStateTransitionMatrix()
{
    return F_;
}


Eigen::MatrixXd DiscretizedKinematicModel::getNoiseCovarianceMatrix()
{
    return Q_;
}


bool DiscretizedKinematicModel::setProperty(const std::string& property)
{
    if (property == "tick")
    {
        if (estimate_period_)
        {
            // Evaluate elapsed time and reset matrices F_ and Q_
            std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();

            if (last_time_set_)
            {
                std::chrono::duration<double, std::milli> delta_chrono = now - last_time_;
                double delta = delta_chrono.count() / 1000.0;

                evaluateStateTransitionMatrix(delta);
                evaluateNoiseCovarianceMatrix(delta);
            }

            last_time_ = now;
            last_time_set_ = true;
        }
        return true;
    }
    else if (property == "reset")
    {
        last_time_set_ = false;
    }

    return false;
}
