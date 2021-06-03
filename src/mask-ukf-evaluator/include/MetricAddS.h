/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef METRICADDS_H
#define METRICADDS_H

#include <Eigen/Dense>

#include <Metric.h>


class MetricAddS : public Metric
{
public:
    MetricAddS();

    ~MetricAddS();

    Eigen::VectorXd evaluate(const Eigen::Transform<double, 3, Eigen::Affine>& estimate, const Eigen::Transform<double, 3, Eigen::Affine>& ground_truth, const Eigen::MatrixXd& model) override;
};

#endif /* METRICADDS_H */
