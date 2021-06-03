/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef METRICEMPTY_H
#define METRICEMPTY_H

#include <Eigen/Dense>

#include <Metric.h>


class MetricEmpty : public Metric
{
public:
    MetricEmpty();

    virtual ~MetricEmpty();

    Eigen::VectorXd evaluate(const Eigen::Transform<double, 3, Eigen::Affine>& estimate, const Eigen::Transform<double, 3, Eigen::Affine>& ground_truth, const Eigen::MatrixXd& model);
};

#endif /* METRICEMPTY_H */
