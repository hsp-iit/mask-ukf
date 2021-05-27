/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef YCBVIDEOCAMERA_H
#define YCBVIDEOCAMERA_H

#include <Camera.h>

#include <Eigen/Dense>


class YcbVideoCamera : public Camera
{
public:

    YcbVideoCamera(const std::string& path, const std::size_t& width, const std::size_t& height, const std::string& config_context);

    ~YcbVideoCamera();

    bool freeze() override;

    bool reset() override;

    bool setFrame(const std::size_t& number) override;

    std::pair<bool, Eigen::Transform<double, 3, Eigen::Affine>> getCameraPose(const bool& blocking) override;

    std::pair<bool, cv::Mat> getRgbImage(const bool& blocking) override;

    std::pair<bool, Eigen::MatrixXf> getDepthImage(const bool& blocking) override;

private:
    std::string composeFileName(const std::size_t& index, const std::size_t& number_digits);

    bool checkImage(const std::size_t& index);

    int head_ = 0;

    std::string path_rgb_images_;

    std::string path_depth_images_;

    std::size_t number_of_digits_;

    const std::string log_ID_ = "YcbVideoCamera";
};

#endif /* YCBVIDEOCAMERANRT_H */
