/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef DISCRETIZEDKINEMATICMODEL_H
#define DISCRETIZEDKINEMATICMODEL_H

#include <BayesFilters/LinearStateModel.h>

#include <Eigen/Dense>

#include <chrono>


class DiscretizedKinematicModel : public bfl::LinearStateModel
{
public:
    DiscretizedKinematicModel
    (
        const double sigma_x,
        const double sigma_y,
        const double sigma_z,
        const double sigma_phi,
        const double sigma_theta,
        const double sigma_psi,
        const double period,
        const bool estimate_period
    );

    virtual ~DiscretizedKinematicModel();

    Eigen::MatrixXd getStateTransitionMatrix() override;

    bool setProperty(const std::string& property) override;

    Eigen::MatrixXd getNoiseCovarianceMatrix();

    std::pair<std::size_t, std::size_t> getOutputSize() const override;

protected:
    void evaluateStateTransitionMatrix(const double T);

    void evaluateNoiseCovarianceMatrix(const double T);

    /**
     * State transition matrix.
     */
    Eigen::MatrixXd F_;

    /**
     * Noise covariance matrix.
     */
    Eigen::MatrixXd Q_;

    /**
     * Squared power spectral densities
     */
    Eigen::VectorXd sigma_position_;

    Eigen::VectorXd sigma_orientation_;

    std::chrono::steady_clock::time_point last_time_;

    bool last_time_set_ = false;

    bool estimate_period_ = false;
};

#endif /* DISCRETIZEDKINEMATICMODEL_H */
