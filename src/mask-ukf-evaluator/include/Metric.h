/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef METRIC_H
#define METRIC_H

#include <Eigen/Dense>


class Metric
{
public:
    Metric();

    virtual ~Metric();

    virtual Eigen::VectorXd evaluate(const Eigen::Transform<double, 3, Eigen::Affine>& estimate, const Eigen::Transform<double, 3, Eigen::Affine>& ground_truth, const Eigen::MatrixXd& model) = 0;

    virtual Eigen::VectorXd evaluate(const Eigen::VectorXd& estimate, const Eigen::VectorXd& ground_truth, const Eigen::Transform<double, 3, Eigen::Affine>& ground_truth_pose);
};

#endif /* METRIC_H */
