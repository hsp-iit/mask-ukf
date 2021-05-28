/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef CORRECTION_H
#define CORRECTION_H

#include <BayesFilters/AdditiveMeasurementModel.h>

#include <ObjectMeasurements.h>
#include <SUKFCorrection.h>


class Correction : public SUKFCorrection
{
public:
    Correction
    (
        std::unique_ptr<bfl::AdditiveMeasurementModel> meas_model,
        /**
         * Unscented transform parameters
         */
        const std::size_t state_size,
        const double      alpha,
        const double      beta,
        const double      kappa,
        /**
         * Subsize, J, of measurement vector y in R^M such that
         * M = k * J for some positive integer k
         */
        const std::size_t meas_sub_size
     ) noexcept;

    void correctStep(const bfl::GaussianMixture& pred_state, bfl::GaussianMixture& corr_state) override;
};

#endif /* CORRECTION_H */
