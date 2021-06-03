/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <Metric.h>

using namespace Eigen;


Metric::Metric()
{ }


Metric::~Metric()
{ }


Eigen::VectorXd Metric::evaluate(const Eigen::VectorXd& estimate, const Eigen::VectorXd& ground_truth, const Eigen::Transform<double, 3, Eigen::Affine>& ground_truth_pose)
{
    return VectorXd();
}
