/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef SINGLEVIDEORESULT_H
#define SINGLEVIDEORESULT_H

#include <Eigen/Dense>

class SingleVideoResult
{
public:
    SingleVideoResult(const std::string& object_name, const std::string& video_name, const std::string& result_path);

    Eigen::Transform<double, 3, Eigen::Affine> getObjectPose(const std::size_t& frame_id) const;

    Eigen::Transform<double, 3, Eigen::Affine> getGroundTruthPose(const std::size_t& frame_id) const;

    Eigen::VectorXd getObjectVelocity(const std::size_t& frame_id) const;

    Eigen::VectorXd getGroundTruthVelocity(const std::size_t& frame_id, const std::size_t& frame_id_0);

    double getFPS(const std::size_t& frame_id) const;

    std::string getVideoName() const;

    std::size_t getNumberFrames() const;

private:
    std::pair<bool, Eigen::MatrixXd> readDataFromFile(const std::string& filename, const std::size_t num_fields);

    Eigen::MatrixXd padMissingFrames(const Eigen::MatrixXd& data, const std::size_t& expected_length);

    Eigen::Transform<double, 3, Eigen::Affine> makeTransform(const Eigen::VectorXd pose) const;

    Eigen::MatrixXd estimate_;

    Eigen::MatrixXd ground_truth_;

    Eigen::MatrixXd fps_;

    Eigen::VectorXd last_v_;

    const std::string video_name_;

    const std::string log_ID_ = "[SingleVideoResult]";
};

#endif /* SINGLEVIDEORESULT_H */
