/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <InitGroundTruth.h>

#include <fstream>
#include <iostream>
#include <vector>

using namespace Eigen;


InitGroundTruth::InitGroundTruth
(
    const std::string& path,
    const std::string& object_name,
    const std::size_t& frame_index,
    const MatrixXd& initial_covariance
) :
    index_(frame_index)
{
    /* Compose root path. */
    std::string root = path;
    if (root.back() != '/')
        root += '/';

    /* Compose path containing ground truth data. */
    std::string file_name = root + "gt_" + object_name.substr(object_name.find("_") + 1) + "/data.log";

    /* Load data. */
    bool valid_data = false;
    data_shift_ = 0;
    std::tie(valid_data, data_) = readStateFromFile(file_name, 9);
    if (!valid_data)
    {
        // Try with a different format
        file_name = root + "gt/data.log";
        std::tie(valid_data, data_) = readStateFromFile(file_name, 10);

        data_shift_ = 1;

        if (!valid_data)
        {
            std::string error = log_ID_ + "::ctor. Error: cannot open ground truth data (" + file_name + ")";
            throw(std::runtime_error(error));
        }
    }

    /* Extract initial pose and convert to x-y-z-Euler(Z, Y, X). */
    initial_pose_ = VectorXd::Zero(12);
    initial_pose_.head<3>() = data_.col(frame_index).segment<3>(2 + data_shift_);
    AngleAxisd angle_axis(data_.col(frame_index)(8 + data_shift_), data_.col(frame_index).segment<3>(5 + data_shift_));
    initial_pose_.tail<3>() = angle_axis.toRotationMatrix().eulerAngles(2, 1, 0);

    initial_covariance_ = initial_covariance;

    initial_pose_.head<3>()(0) += 0.05;
    initial_pose_.head<3>()(1) += 0.05;
    initial_pose_.head<3>()(2) += 0.05;
    initial_pose_.tail<3>()(0) += 10.0 * M_PI / 180;
    initial_pose_.tail<3>()(1) += 10.0 * M_PI / 180;
    initial_pose_.tail<3>()(2) += 10.0 * M_PI / 180;
}


InitGroundTruth::~InitGroundTruth() noexcept
{ }


Eigen::VectorXd InitGroundTruth::getInitialPose()
{
    return initial_pose_;
}


Eigen::MatrixXd InitGroundTruth::getInitialCovariance()
{
    return initial_covariance_;
}


void InitGroundTruth::step()
{
    index_++;
    initial_pose_ = VectorXd::Zero(12);
    initial_pose_.head<3>() = data_.col(index_).segment<3>(2 + data_shift_);
    AngleAxisd angle_axis(data_.col(index_)(8 + data_shift_), data_.col(index_).segment<3>(5 + data_shift_));
    initial_pose_.tail<3>() = angle_axis.toRotationMatrix().eulerAngles(2, 1, 0);

    initial_pose_.head<3>()(0) += 0.05;
    initial_pose_.head<3>()(1) += 0.05;
    initial_pose_.head<3>()(2) += 0.05;
    initial_pose_.tail<3>()(0) += 10.0 * M_PI / 180;
    initial_pose_.tail<3>()(1) += 10.0 * M_PI / 180;
    initial_pose_.tail<3>()(2) += 10.0 * M_PI / 180;
}


std::pair<bool, MatrixXd> InitGroundTruth::readStateFromFile(const std::string& filename, const std::size_t num_fields)
{
    MatrixXd data;

    std::ifstream istrm(filename);

    if (!istrm.is_open())
    {
        return std::make_pair(false, MatrixXd(0,0));
    }

    std::vector<std::string> istrm_strings;
    std::string line;
    while (std::getline(istrm, line))
    {
        istrm_strings.push_back(line);
    }

    data.resize(num_fields, istrm_strings.size());
    std::size_t found_lines = 0;
    for (auto line : istrm_strings)
    {
        std::size_t found_fields = 0;
        std::string number_str;
        std::istringstream iss(line);

        while (iss >> number_str)
        {
            std::size_t index = (num_fields * found_lines) + found_fields;
            *(data.data() + index) = std::stod(number_str);
            found_fields++;
        }
        if (num_fields != found_fields)
        {
            return std::make_pair(false, MatrixXd(0,0));
        }
        found_lines++;
    }

    return std::make_pair(true, data);
}
