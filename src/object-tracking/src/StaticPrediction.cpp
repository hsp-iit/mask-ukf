/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <StaticPrediction.h>

using namespace bfl;


StaticPrediction::StaticPrediction() :
    model_placeholder_(std::unique_ptr<StaticModel>(new StaticModel()))
{ }


StaticPrediction::~StaticPrediction()
{ }


StateModel& StaticPrediction::getStateModel() noexcept
{
    return *model_placeholder_;
}


void StaticPrediction::predictStep(const GaussianMixture& prev_state, GaussianMixture& pred_state)
{
    // The predicted state corresponds to the corrected state at the previous step.
    // This is used for the implementation of ICP.
    pred_state = prev_state;
}
