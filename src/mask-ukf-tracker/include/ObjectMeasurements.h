/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef OBJECTMEASUREMENTS_H
#define OBJECTMEASUREMENTS_H

#include <BayesFilters/AdditiveMeasurementModel.h>
#include <BayesFilters/Data.h>
#include <BayesFilters/VectorDescription.h>

#include <Camera.h>
#include <PointCloudPrediction.h>
#include <PointCloudSegmentation.h>

#include <Eigen/Dense>

#include <memory>


class ObjectMeasurements : public bfl::AdditiveMeasurementModel
{
public:
    ObjectMeasurements(std::unique_ptr<Camera> camera, std::shared_ptr<PointCloudSegmentation> segmentation, std::shared_ptr<PointCloudPrediction> prediction, const Eigen::Ref<const Eigen::Matrix3d>& visual_noise_covariance, const std::string& depth_fetch_mode);

    ~ObjectMeasurements();

    std::pair<bool, bfl::Data> measure(const bfl::Data& data = bfl::Data()) const;

    bool freeze(const bfl::Data& data = bfl::Data()) override;

    bfl::VectorDescription getInputDescription() const override;

    bfl::VectorDescription getMeasurementDescription() const override;

    std::pair<bool, bfl::Data> predictedMeasure(const Eigen::Ref<const Eigen::MatrixXd>& current_states) const override;

    std::pair<bool, bfl::Data> innovation(const bfl::Data& predicted_measurements, const bfl::Data& measurements) const override;

    std::pair<bool, Eigen::MatrixXd> getNoiseCovarianceMatrix() const override;

    bool setProperty(const std::string& property) override;

    void enableOutlierRejection(const bool& enable);

protected:

    bool getDepth();

    virtual void reset();

    /**
     * Local copy of depht image.
     */
    Eigen::MatrixXf depth_;

    std::string depth_fetch_mode_;

    bool depth_initialized_ = false;

    /**
     * Local copy of measurements.
     * A vector of size 3 * L with L the number of points in set.
     */
    Eigen::MatrixXd measurement_;

    std::unique_ptr<Camera> camera_;

    std::shared_ptr<PointCloudSegmentation> segmentation_;

    std::shared_ptr<PointCloudPrediction> prediction_;

    Eigen::Matrix3d visual_noise_covariance_;

    bool measurements_available_ = true;

    bool outlier_rejection_ = false;

    std::vector<std::string> log_file_names(const std::string& prefix_path, const std::string& prefix_name) override
    {
        return {prefix_path + "/" + prefix_name + "_measurements"};
    }
};

#endif /* OBJECTMEASUREMENTS_H */
