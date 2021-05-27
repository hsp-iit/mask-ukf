/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef STATICPREDICTION_H
#define STATICPREDICTION_H

#include <BayesFilters/GaussianMixture.h>
#include <BayesFilters/GaussianPrediction.h>
#include <BayesFilters/StateModel.h>

#include <memory>


class StaticModel : public bfl::StateModel
{
public:
    StaticModel()
    { };

    virtual ~StaticModel()
    { };

    void propagate(const Eigen::Ref<const Eigen::MatrixXd>& cur_states, Eigen::Ref<Eigen::MatrixXd> prop_states) override
    { };

    void motion(const Eigen::Ref<const Eigen::MatrixXd>& cur_states, Eigen::Ref<Eigen::MatrixXd> mot_states) override
    { };

    bool setProperty(const std::string& property) override
    {
        return false;
    };

    std::pair<std::size_t, std::size_t> getOutputSize() const override
    {
        return std::make_pair(9, 3);
    };
};

class StaticPrediction : public bfl::GaussianPrediction
{
public:
    StaticPrediction();

    virtual ~StaticPrediction();

    bfl::StateModel& getStateModel() noexcept override;

protected:
    void predictStep(const bfl::GaussianMixture& prev_state, bfl::GaussianMixture& pred_state) override;

private:
    std::unique_ptr<StaticModel> model_placeholder_;
};

#endif /* STATICPREDICTION_H */
