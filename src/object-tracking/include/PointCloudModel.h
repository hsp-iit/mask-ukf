/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef POINTCLOUDMODEL_H
#define POINTCLOUDMODEL_H

#include <BayesFilters/AdditiveMeasurementModel.h>

#include <PointCloudPrediction.h>

#include <Eigen/Dense>

#include <memory>


class PointCloudModel : public bfl::AdditiveMeasurementModel
{
public:
    PointCloudModel(std::unique_ptr<PointCloudPrediction> prediction, const Eigen::Ref<const Eigen::Matrix3d>& noise_covariance_matrix, const Eigen::Ref<const Eigen::Matrix3d>& tactile_noise_covariance_matrix);

    std::pair<bool, bfl::Data> predictedMeasure(const Eigen::Ref<const Eigen::MatrixXd>& current_states) const override;

    std::pair<bool, bfl::Data> innovation(const bfl::Data& predicted_measurements, const bfl::Data& measurements) const override;

    std::pair<bool, Eigen::MatrixXd> getNoiseCovarianceMatrix() const override;

    std::pair<bool, Eigen::MatrixXd> getTactileNoiseCovarianceMatrix() const;

protected:
    std::unique_ptr<PointCloudPrediction> prediction_;

    Eigen::Matrix3d model_noise_covariance_;

    Eigen::Matrix3d tactile_model_noise_covariance_;

    std::vector<std::string> log_file_names(const std::string& prefix_path, const std::string& prefix_name) override
    {
        return {prefix_path + "/" + prefix_name + "_measurements"};
    }
};

#endif /* POINTCLOUDMODEL_H */
