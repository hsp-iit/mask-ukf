/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef MASKSEGMENTATION_H
#define MASKSEGMENTATION_H

#include <Eigen/Dense>

#include <Camera.h>
#include <PointCloudSegmentation.h>


class MaskSegmentation : public PointCloudSegmentation
{
public:

    MaskSegmentation(const std::string& path, const std::string& mask_name, const std::size_t& depth_stride, const std::string& masks_set);

    MaskSegmentation(const std::string& path, const std::string& mask_name, const std::size_t& depth_stride, const std::string& masks_set, const double& simulated_fps);

    ~MaskSegmentation();

    bool freezeSegmentation(Camera& camera) override;

    std::tuple<bool, std::size_t, Eigen::MatrixXd> extractPointCloud(Camera& camera, const Eigen::Ref<const Eigen::MatrixXf>& depth, const double& max_depth) override;

    bool setProperty(const std::string& property) override;

protected:
    std::string composeFileName(const std::size_t& index, const std::size_t& number_digits);

    std::tuple<bool, bool, cv::Mat> getMask(Camera& camera);

    std::size_t depth_stride_;

    std::string mask_name_;

    std::vector<std::pair<int, int>> coordinates_;

    cv::Mat mask_;

    bool mask_initialized_ = false;

    /**
     * Required for simulated fps.
     */

    double fps_;

    const bool simulated_fps_ = false;

    double original_fps_ = 30.0;

    /**
     * Required for offline execution.
     */

    std::string path_mask_images_;

    std::size_t number_of_digits_ = 6;

    int head_ = 0;

    int effective_head_;

    const std::string log_ID_ = "MaskSegmentation";
};

#endif /* MASKSEGMENTATION_H */
