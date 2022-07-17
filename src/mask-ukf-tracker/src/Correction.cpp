/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <Correction.h>

#include <cmath>

using namespace bfl;
using namespace Eigen;


Correction::Correction
(
    std::unique_ptr<AdditiveMeasurementModel> meas_model,
    /**
     * Unscented transform parameters
     */
    const double      alpha,
    const double      beta,
    const double      kappa,
    /**
     * Subsize, J, of measurement vector y in R^M such that
     * M = k * J for some positive integer k
     */
    const std::size_t meas_sub_size
) noexcept :
    SUKFCorrection(std::move(meas_model), alpha, beta, kappa, meas_sub_size, true)
{ }


void Correction::correctStep(const GaussianMixture& pred_state, GaussianMixture& corr_state)
{
    SUKFCorrection::correctStep(pred_state, corr_state);

    // Handle angular components of the state
    corr_state.mean().bottomRows(3) = (std::complex<double>(0.0,1.0) * corr_state.mean().bottomRows(3)).array().exp().arg();
}
