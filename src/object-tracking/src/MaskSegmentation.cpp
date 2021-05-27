/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <MaskSegmentation.h>

#include <fstream>
#include <iostream>

#include <yarp/cv/Cv.h>
#include <yarp/os/Value.h>


using namespace Eigen;
using namespace yarp::cv;
using namespace yarp::os;
using namespace yarp::sig;


MaskSegmentation::MaskSegmentation(const std::string& path, const std::string& mask_name, const std::size_t& depth_stride, const std::string& masks_set) :
    mask_name_(mask_name),
    depth_stride_(depth_stride)
{
    /* Compose root path. */
    std::string root = path;
    if (root.back() != '/')
        root += '/';

    /* Compose path containing mask images. */
    path_mask_images_ = root + "masks/" + masks_set + "/";
}


MaskSegmentation::MaskSegmentation(const std::string& path, const std::string& mask_name, const std::size_t& depth_stride, const std::string& masks_set, const double& simulated_fps) :
    mask_name_(mask_name),
    depth_stride_(depth_stride),
    simulated_fps_(true),
    fps_(simulated_fps)
{
    /* Compose root path. */
    std::string root = path;
    if (root.back() != '/')
        root += '/';

    /* Compose path containing mask images. */
    path_mask_images_ = root + "masks/" + masks_set + "/";
}


MaskSegmentation::~MaskSegmentation()
{}


bool MaskSegmentation::freezeSegmentation(Camera& camera)
{
    // Get camera parameters
    bool valid_parameters = false;
    CameraParameters camera_parameters;
    std::tie(valid_parameters, camera_parameters) = camera.getIntrinsicParameters();
    if (!valid_parameters)
        return false;

    // Get binary mask
    bool received_mask = false;
    bool valid_mask = false;
    cv::Mat mask;
    std::tie(received_mask, valid_mask, mask) = getMask(camera);

    // A binary mask has been received and it is valid
    if (received_mask)
    {
        if (valid_mask)
        {
            mask_initialized_ = true;

            mask_ = mask.clone();
        }
        // A binary mask has been received but it is empty (i.e. no detection)
        else
        {
            mask_initialized_ = false;

            coordinates_.clear();
        }
    }

    // As soon as a mask has been received the coordinates are extracted
    // If a mask has not been received but the last one was valid,
    // 'mask_initialized_' will be true and old coordinates are used
    // If an empty mask was received 'mask_initialized_' will be false
    // and coordinates_ will be empty
    // The variable coordinates_ is used in method extractPointCloud()
    if(mask_initialized_)
    {
        // Find non zero coordinates
        cv::Mat non_zero_coordinates;
        cv::findNonZero(mask_, non_zero_coordinates);

        // Fill coordinates vector
        coordinates_.clear();
        for (std::size_t i = 0; i < non_zero_coordinates.total(); i+= depth_stride_)
        {
            cv::Point& p = non_zero_coordinates.at<cv::Point>(i);
            coordinates_.push_back(std::make_pair(p.x, p.y));
        }
    }

    return true;
}


std::tuple<bool, std::size_t, MatrixXd> MaskSegmentation::extractPointCloud(Camera& camera, const Ref<const MatrixXf>& depth, const double& max_depth)
{
    // Find valid points according to depth
    VectorXi valid_points(coordinates_.size());
    for (std::size_t i = 0; i < valid_points.size(); i++)
    {
        valid_points(i) = 0;

        float depth_u_v = depth(coordinates_[i].second, coordinates_[i].first);

        if ((depth_u_v > 0) && (depth_u_v < max_depth))
            valid_points(i) = 1;
    }

    // Check if there are valid points
    std::size_t num_valids = valid_points.sum();

    // Get camera parameters
    bool valid_parameters = false;
    CameraParameters camera_parameters;
    std::tie(valid_parameters, camera_parameters) = camera.getIntrinsicParameters();
    if (!valid_parameters)
        return std::make_tuple(false, 0, MatrixXd());

    // Get deprojection matrix
    bool valid_deprojection_matrix = false;
    MatrixXd deprojection_matrix;
    std::tie(valid_deprojection_matrix, deprojection_matrix) = camera.getDeprojectionMatrix();
    if (!valid_deprojection_matrix)
        return std::make_tuple(false, 0, MatrixXd());

    // Store only valid points
    MatrixXd points(3, num_valids);
    for (int i = 0, j = 0; i < coordinates_.size(); i++)
    {
        if(valid_points(i) == 1)
        {
            const int& u = coordinates_[i].first;
            const int& v = coordinates_[i].second;

            points.col(j) = deprojection_matrix.col(u * camera_parameters.height + v) * depth(v, u);

            j++;
        }
    }

    // When coordinates_ is empty the resulting point cloud will be empty too
    return std::make_tuple(true, coordinates_.size(), points);
}


bool MaskSegmentation::setProperty(const std::string& property)
{
    bool valid = PointCloudSegmentation::setProperty(property);

    if (property == "reset")
        head_ = 1;

    return valid;
}



std::tuple<bool, bool, cv::Mat> MaskSegmentation::getMask(Camera& camera)
{
    cv::Mat mask;

    head_++;

    // Move head to the next frame
    if (simulated_fps_)
    {
        if ((head_ -1) % static_cast<int>(original_fps_ / fps_) == 0)
            effective_head_ = head_;
    }
    else
        effective_head_ = head_;

    std::string file_name = path_mask_images_ + mask_name_ + "_" + composeFileName(effective_head_, number_of_digits_) + ".png";
    mask = cv::imread(file_name, cv::IMREAD_COLOR);

    if (mask.empty())
    {
        // Detection not available for this frame
        return std::make_tuple(true, false, cv::Mat());
    }

    cv::cvtColor(mask, mask, cv::COLOR_BGR2GRAY);

    CameraParameters parameters;
    std::tie(std::ignore, parameters) = camera.getIntrinsicParameters();
    cv::resize(mask, mask, cv::Size(parameters.width, parameters.height));

    cv::Mat non_zero_coordinates;
    cv::findNonZero(mask, non_zero_coordinates);

    if (non_zero_coordinates.total() == 0)
        // Mask available but empty
        return std::make_tuple(true, false, cv::Mat());

    // Mask available and non-empty
    return std::make_tuple(true, true, mask);
}


std::string MaskSegmentation::composeFileName(const std::size_t& index, const std::size_t& number_digits)
{
    std::ostringstream ss;
    ss << std::setw(number_digits) << std::setfill('0') << index;
    return ss.str();
}
