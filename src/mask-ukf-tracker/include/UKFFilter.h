/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef UKFFILTER_H
#define UKFFILTER_H

#include <BayesFilters/Gaussian.h>
#include <BayesFilters/GaussianCorrection.h>
#include <BayesFilters/GaussianFilter.h>
#include <BayesFilters/GaussianPrediction.h>
#include <BayesFilters/MeasurementModel.h>

#include <InitGroundTruth.h>
#include <ObjectMeasurements.h>

#include <chrono>
#include <memory>


class UKFFilter : public bfl::GaussianFilter
{
public:
    UKFFilter
    (
        std::unique_ptr<InitGroundTruth> initialization,
        std::unique_ptr<bfl::GaussianPrediction> prediction,
        std::unique_ptr<bfl::GaussianCorrection> correction
    );

    virtual ~UKFFilter();

    bool run_condition() override;

    bool initialization_step() override;

protected:
    std::vector<std::string> log_file_names(const std::string& prefix_path, const std::string& prefix_name) override;

    void filtering_step() override;

private:
    void startTimeCount();

    double stopTimeCount();

    std::chrono::steady_clock::time_point std_time_0_;

    bfl::Gaussian pred_belief_;

    bfl::Gaussian corr_belief_;

    Eigen::VectorXd last_ground_truth_;

    std::unique_ptr<InitGroundTruth> initialization_;

    std::size_t keyframe_counter_ = 1;

    bool pause_ = false;

    const std::string log_ID_ = "UKFFilter";
};

#endif /* UKFFILTER_H */
