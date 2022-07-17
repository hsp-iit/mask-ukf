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
#include <BayesFilters/VectorDescription.h>

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

    bfl::VectorDescription getInputDescription() override
    {
        // 9 linear components (x, y, z, x_dot, y_dot, z_dot, yaw_dot, pitch_dot, roll_dot)
        // 3 angular components (yaw, pitch, roll)
        // 12 noise components
        return bfl::VectorDescription(9, 3, 12, bfl::VectorDescription::CircularType::Euler);
    };

    bfl::VectorDescription getStateDescription() override
    {
        // 9 linear components (x, y, z, x_dot, y_dot, z_dot, yaw_dot, pitch_dot, roll_dot)
        // 3 angular components (yaw, pitch, roll)
        // 0 noise components
        return bfl::VectorDescription(9, 3, 0, bfl::VectorDescription::CircularType::Euler);
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
