/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef OBJECTPOINTCLOUDPREDICTION_H
#define OBJECTPOINTCLOUDPREDICTION_H

#include <Eigen/Dense>

#include <PointCloudPrediction.h>
#include <kdTree.h>

#include <BayesFilters/ParticleSet.h>
#include <BayesFilters/ParticleSetInitialization.h>

#include <memory>
#include <string>


class ObjectPointCloudPrediction : public PointCloudPrediction
{
public:
    ObjectPointCloudPrediction(const std::string& point_cloud_filename);

    ~ObjectPointCloudPrediction();

    std::pair<bool, Eigen::MatrixXd> predictPointCloud(ConstMatrixXdRef state, ConstVectorXdRef meas) override;

    std::pair<bool, Eigen::MatrixXd> evaluateDistances(ConstMatrixXdRef state, ConstVectorXdRef meas) override;

    Eigen::MatrixXd evaluateModel(const Eigen::Transform<double, 3, Eigen::Affine>& object_pose);

private:
    std::pair<bool, Eigen::MatrixXd> loadPointCloudFromFile(const std::string& filename);

    Eigen::MatrixXd cloud_;

    const std::string log_ID_ = "ObjectPointCloudPrediction";
};

#endif
