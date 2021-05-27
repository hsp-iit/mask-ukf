/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <PointCloudModel.h>

using namespace bfl;
using namespace Eigen;

PointCloudModel::PointCloudModel
(
    std::unique_ptr<PointCloudPrediction> prediction,
    const Eigen::Ref<const Eigen::Matrix3d>& noise_covariance_matrix,
    const Eigen::Ref<const Eigen::Matrix3d>& tactile_noise_covariance_matrix
    ) :
    prediction_(std::move(prediction)),
    model_noise_covariance_(noise_covariance_matrix),
    tactile_model_noise_covariance_(tactile_noise_covariance_matrix)
{ }


std::pair<bool, bfl::Data> PointCloudModel::predictedMeasure(const Eigen::Ref<const Eigen::MatrixXd>& cur_states) const
{
    bool valid_measure;
    bfl::Data measurement;
    std::tie(valid_measure, measurement) = measure();

    if (!valid_measure)
        return std::make_pair(false, Data());

    bool valid_prediction;
    MatrixXd prediction;
    std::tie(valid_prediction, prediction) = prediction_->predictPointCloud(cur_states, any::any_cast<MatrixXd>(measurement).col(0));

    if (!valid_prediction)
        return std::make_pair(false, Data());

    return std::make_pair(true, prediction);
}


std::pair<bool, bfl::Data> PointCloudModel::innovation(const bfl::Data& predicted_measurements, const bfl::Data& measurements) const
{
    MatrixXd innovation = -(any::any_cast<MatrixXd>(predicted_measurements).colwise() - any::any_cast<MatrixXd>(measurements).col(0));

    return std::make_pair(true, std::move(innovation));
}


std::pair<bool, MatrixXd> PointCloudModel::getNoiseCovarianceMatrix() const
{
    return std::make_pair(true, model_noise_covariance_);
}


std::pair<bool, MatrixXd> PointCloudModel::getTactileNoiseCovarianceMatrix() const
{
    return std::make_pair(true, tactile_model_noise_covariance_);
}
