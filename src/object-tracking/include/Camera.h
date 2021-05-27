/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef CAMERA_H
#define CAMERA_H

#include <CameraParameters.h>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>

#include <string>


class Camera
{
public:
    virtual ~Camera();

    virtual bool initialize();

    virtual bool freeze();

    virtual bool reset();

    virtual bool setFrame(const std::size_t& number);

    virtual std::pair<bool, Eigen::Transform<double, 3, Eigen::Affine>> getCameraPose(const bool& blocking) = 0;

    virtual std::pair<bool, cv::Mat> getRgbImage(const bool& blocking) = 0;

    virtual std::pair<bool, Eigen::MatrixXf> getDepthImage(const bool& blocking) = 0;

    virtual std::pair<bool, CameraParameters> getIntrinsicParameters() const;

    virtual std::pair<bool, Eigen::MatrixXd> getDeprojectionMatrix();

protected:
    virtual bool evalDeprojectionMatrix();

    CameraParameters parameters_;

    Eigen::MatrixXd deprojection_matrix_;

    bool deprojection_matrix_initialized_ = false;

    const std::string log_ID_base_ = "Camera";
};

#endif /* CAMERA_H */
