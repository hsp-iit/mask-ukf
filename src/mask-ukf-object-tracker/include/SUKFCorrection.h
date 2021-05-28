/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef SUKFCORRECTION__H
#define SUKFCORRECTION__H

#include <BayesFilters/AdditiveMeasurementModel.h>
#include <BayesFilters/GaussianCorrection.h>
#include <BayesFilters/GaussianMixture.h>
#include <BayesFilters/sigma_point.h>

#include <Eigen/Dense>


/**
 * This class implements the algorithm:
 * Barfoot, T., McManus, C. (2011),
 * 'A Serial Approach to Handling High-Dimensional Measurements in the Sigma-Point Kalman Filter.',
 * Science and Systems VII,
 * MIT Press
 */


class SUKFCorrection : public bfl::GaussianCorrection
{
public:
    /**
     * If input argument use_reduced_noise_covariance_matrix is set to true,
     * when the algorithm calls measurement_model_.getNoiseCovarianceMatrix()
     * it expects to receive a noise covariance matrix of size measurement_sub_size_ x measurement_sub_size_
     * and uses it for all the sub-vectors beloning to the current measurement.
     * If set to false, the algorithms expects to receive the complete noise covariance matrix
     * consisting in (M / meas_sub_size) diagonal blocks of size measurement_sub_size_ x measurement_sub_size_
     * where M is the size of the current measurement.
     * The input argument meas_sub_size is the sub-size, J, of the
     * measurement vector y in R^M such that M = k * J for some positive integer k.
     */
    SUKFCorrection(std::unique_ptr<bfl::AdditiveMeasurementModel> measurement_model, const std::size_t state_size, const double alpha, const double beta, const double kappa, const std::size_t measurement_sub_size, const bool use_reduced_noise_covariance_matrix) noexcept;

    SUKFCorrection(SUKFCorrection&& sukf_correction) noexcept;

    virtual ~SUKFCorrection() noexcept { };

    bfl::MeasurementModel& getMeasurementModel() override;

    std::pair<bool, Eigen::VectorXd> getLikelihood() override;

protected:
    void correctStep(const bfl::GaussianMixture& pred_state, bfl::GaussianMixture& corr_state) override;

    virtual Eigen::MatrixXd getNoiseCovarianceMatrix(const std::size_t index);

private:
    std::unique_ptr<bfl::MeasurementModel> measurement_model_;

    Eigen::MatrixXd propagated_sigma_points_;

    Eigen::MatrixXd innovations_;

    /**
     * Unscented transform weight.
     */
    bfl::sigma_point::UTWeight ut_weight_;

    std::size_t measurement_sub_size_;

    bool use_reduced_noise_covariance_matrix_;
};

#endif /* SUKFCORRECTION__H */
