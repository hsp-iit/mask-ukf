/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <MetricRMS.h>

using namespace Eigen;


MetricRMS::MetricRMS()
{ }


MetricRMS::~MetricRMS()
{ }


VectorXd MetricRMS::evaluate(const Eigen::Transform<double, 3, Eigen::Affine>& estimate, const Eigen::Transform<double, 3, Eigen::Affine>& ground_truth, const Eigen::MatrixXd& model)
{
    VectorXd errors(2);

    if (((estimate.translation()(0) == std::numeric_limits<double>::infinity()) &&
         (estimate.translation()(1) == std::numeric_limits<double>::infinity()) &&
         (estimate.translation()(2) == std::numeric_limits<double>::infinity())) ||
        ((estimate.translation()(0) == 0.0) &&
         (estimate.translation()(1) == 0.0) &&
         (estimate.translation()(2) == 0.0)))
    {
        // Warning: this check is required since some DenseFusion frames might be missing.
        // When missing, they are identified as having an infinite estimate or all zeros
        // In such cases, we decided to assign a null error for the evaluation of the RMSE
        errors(0) = 0.0;
        errors(1) = 0.0;

        return errors;
    }

    errors(0) = (estimate.translation() - ground_truth.translation()).squaredNorm();
    errors(1) = std::pow(AngleAxisd((estimate.rotation().transpose() * ground_truth.rotation())).angle(), 2);

    return errors;
}
