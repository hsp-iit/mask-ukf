/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef INITGROUNDTRUTH_H
#define INITGROUNDTRUTH_H

#include <Eigen/Dense>


class InitGroundTruth
{
public:
    InitGroundTruth(const std::string& path, const std::string& object_name, const std::size_t& frame_index, const Eigen::MatrixXd& initial_covariance);

    virtual ~InitGroundTruth() noexcept;

    Eigen::VectorXd getInitialPose();

    Eigen::MatrixXd getInitialCovariance();

protected:
    std::pair<bool, Eigen::MatrixXd> readStateFromFile(const std::string& filename, const std::size_t num_fields);

    Eigen::MatrixXd data_;

    std::size_t data_shift_;

    Eigen::VectorXd initial_pose_;

    Eigen::MatrixXd initial_covariance_;

    std::size_t index_ = 0;

    std::string log_ID_ = "InitGroundTruth";
};

#endif /* INITGROUNDTRUTH_H */
