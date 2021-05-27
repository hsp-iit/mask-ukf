/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef CORRECTIONICP_H
#define CORRECTIONICP_H

#include <BayesFilters/AdditiveMeasurementModel.h>
#include <BayesFilters/GaussianCorrection.h>

#include <Eigen/Dense>

#include <ObjectMeasurements.h>


class CorrectionICP : public bfl::GaussianCorrection
{
public:
    CorrectionICP(std::unique_ptr<bfl::AdditiveMeasurementModel> measurement_model, const std::string& point_cloud_filename);

    virtual ~CorrectionICP();

    void correctStep(const bfl::GaussianMixture& pred_state, bfl::GaussianMixture& corr_state) override;

    bfl::MeasurementModel& getMeasurementModel() override;

private:
    std::pair<bool, Eigen::MatrixXd> loadPointCloudFromFile(const std::string& filename);

    std::unique_ptr<bfl::AdditiveMeasurementModel> measurement_model_;

    Eigen::MatrixXd model_;

    const std::string log_ID_ = "CorrectionICP";
};

#endif /* CORRECTIONICP_H */
