/*
f * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <UKFFilter.h>

#include <BayesFilters/utils.h>

using namespace bfl;
using namespace Eigen;


UKFFilter::UKFFilter
(
    std::unique_ptr<InitGroundTruth> initialization,
    std::unique_ptr<GaussianPrediction> prediction,
    std::unique_ptr<GaussianCorrection> correction
) :
    GaussianFilter(std::move(prediction), std::move(correction)),
    initialization_(std::move(initialization))
{
}


UKFFilter::~UKFFilter()
{
    disable_log();
}


bool UKFFilter::run_condition()
{
    return true;
}


bool UKFFilter::initialization_step()
{
    pred_belief_.resize(12);
    corr_belief_.resize(12);

    pred_belief_.mean() = initialization_->getInitialPose();
    pred_belief_.covariance() = initialization_->getInitialCovariance();

    return true;
}


std::vector<std::string> UKFFilter::log_file_names(const std::string& prefix_path, const std::string& prefix_name)
{
    return  {prefix_path + "/" + prefix_name + "_estimate",
             prefix_path + "/" + prefix_name + "_ground_truth",
             prefix_path + "/" + prefix_name + "_fps"};
}


void UKFFilter::filtering_step()
{
    // When there are no more images in the dataset, teardown() is invoked
    if(!correction().getMeasurementModel().setProperty("measurements_available"))
    {
        teardown();
        return;
    }

    startTimeCount();

    if (step_number() == 0)
        correction().freeze_measurements(pred_belief_);
    else
        correction().freeze_measurements(corr_belief_);

    if (step_number() != 0)
        prediction().predict(corr_belief_, pred_belief_);
    correction().correct(pred_belief_, corr_belief_);

    // Tick the state model
    prediction().getStateModel().setProperty("tick");

    // Evaluate fps
    double execution_time = stopTimeCount();
    double fps = (execution_time != 0) ? (1 / execution_time) : -1;

    // Send data
    VectorXd estimate(14);
    estimate.head<3>() = corr_belief_.mean().head<3>();
    AngleAxisd angle_axis(AngleAxisd(corr_belief_.mean(9), Vector3d::UnitZ()) *
                          AngleAxisd(corr_belief_.mean(10), Vector3d::UnitY()) *
                          AngleAxisd(corr_belief_.mean(11), Vector3d::UnitX()));
    estimate.segment<3>(3) = angle_axis.axis();
    estimate(6) = angle_axis.angle();
    estimate.segment<3>(7) = corr_belief_.mean().segment<3>(3);
    estimate.segment<3>(10) = corr_belief_.mean().segment<3>(6);
    estimate(13) = keyframe_counter_;

    logger(estimate.transpose(), last_ground_truth_.transpose(), fps);

    // Increase keyframe counter
    keyframe_counter_++;
}


void UKFFilter::startTimeCount()
{
    std_time_0_ = std::chrono::steady_clock::now();
}


double UKFFilter::stopTimeCount()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - std_time_0_).count() / 1000.0;
}
