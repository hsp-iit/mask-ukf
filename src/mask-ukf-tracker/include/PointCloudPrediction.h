/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef POINTCLOUDPREDICTION_H
#define POINTCLOUDPREDICTION_H

#include <Eigen/Dense>

using ConstMatrixXdRef = const Eigen::Ref<const Eigen::MatrixXd>&;
using ConstVectorXdRef = const Eigen::Ref<const Eigen::VectorXd>&;


class PointCloudPrediction
{
public:
    virtual std::pair<bool, Eigen::MatrixXd> predictPointCloud(ConstMatrixXdRef state, ConstVectorXdRef meas) = 0;

    virtual std::pair<bool, Eigen::MatrixXd> evaluateDistances(ConstMatrixXdRef state, ConstVectorXdRef meas) = 0;

    virtual Eigen::MatrixXd evaluateModel(const Eigen::Transform<double, 3, Eigen::Affine>& object_pose) = 0;
};

#endif /* POINTCLOUDPREDICTION_H */
