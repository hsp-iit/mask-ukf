/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef POINTCLOUDSEGMENTATION_H
#define POINTCLOUDSEGMENTATION_H

#include <Camera.h>

#include <Eigen/Dense>


class PointCloudSegmentation
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PointCloudSegmentation();

    virtual ~PointCloudSegmentation();

    virtual bool freezeSegmentation(Camera& camera) = 0;

    virtual std::tuple<bool, std::size_t, Eigen::MatrixXd> extractPointCloud(Camera& camera, const Eigen::Ref<const Eigen::MatrixXf>& depth, const double& max_depth) = 0;

    virtual bool getProperty(const std::string& property) const;

    virtual bool setProperty(const std::string& property);
};

#endif /* POINTCLOUDSEGMENTATION_H */
