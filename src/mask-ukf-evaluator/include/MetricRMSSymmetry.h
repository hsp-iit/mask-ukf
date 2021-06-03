/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef METRICRMSSYMMETRY_H
#define METRICRMSSYMMETRY_H

#include <Eigen/Dense>

#include <Metric.h>


class MetricRMSSymmetry : public Metric
{
public:
    MetricRMSSymmetry(const Eigen::VectorXd& symmetry_axis);

    ~MetricRMSSymmetry();

    Eigen::VectorXd evaluate(const Eigen::Transform<double, 3, Eigen::Affine>& estimate, const Eigen::Transform<double, 3, Eigen::Affine>& ground_truth, const Eigen::MatrixXd& model) override;

    std::pair<Eigen::AngleAxisd, Eigen::AngleAxisd> twistSwingDecomposition(const Eigen::AngleAxisd& rotation, const Eigen::Vector3d& twist_axis);

private:
    Eigen::VectorXd symmetry_axis_;
};

#endif /* METRICRMSSYMMETRY_H */
