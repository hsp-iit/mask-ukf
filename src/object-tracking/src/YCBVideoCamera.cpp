/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <YCBVideoCamera.h>

#include <Eigen/Dense>

#include <fstream>
#include <sstream>

#include <yarp/os/ResourceFinder.h>

using namespace Eigen;
using namespace yarp::os;


YcbVideoCamera::YcbVideoCamera(const std::string& path, const std::size_t& width, const std::size_t& height, const std::string& config_context)
{
    // Camera parameters
    parameters_.width = 320;
    parameters_.height = 240;
    parameters_.fx = 533.4;
    parameters_.fy = 533.75;
    parameters_.cx = 156.495;
    parameters_.cy = 120.655;
    parameters_.initialized = true;

    // Log parameters
    std::cout << log_ID_ + "::ctor. Camera parameters." << std::endl;
    std::cout << log_ID_ + "    - width: " << parameters_.width << std::endl;
    std::cout << log_ID_ + "    - height: " << parameters_.height << std::endl;
    std::cout << log_ID_ + "    - fx: " << parameters_.fx << std::endl;
    std::cout << log_ID_ + "    - fy: " << parameters_.fy << std::endl;
    std::cout << log_ID_ + "    - cx: " << parameters_.cx << std::endl;
    std::cout << log_ID_ + "    - cy: " << parameters_.cy << std::endl;

    /* Compose root path. */
    std::string root = path;
    if (root.back() != '/')
        root += '/';

    /* Compose path containing rgb images. */
    path_rgb_images_ = root + "rgb/";

    /* Compose path containing depth images. */
    path_depth_images_ = root + "depth/";

    /* Determine the structure of the file names by trying several alternatives. */
    std::ifstream in;
    number_of_digits_ = 1;
    bool found_frame_0 = false;
    while(!found_frame_0)
    {
        std::string file_path = path_rgb_images_ + composeFileName(1, number_of_digits_) + ".png";

        in.open(file_path);

        if(in.is_open())
        {
            found_frame_0 = true;
            std::cout << log_ID_ + "::ctor. Found keyframe " << file_path << std::endl;
        }
        else
            number_of_digits_++;

        in.close();
    }
}


YcbVideoCamera::~YcbVideoCamera()
{ }


bool YcbVideoCamera::freeze()
{
    head_++;

    // Test for the next frame being available
    return checkImage(head_);
}


bool YcbVideoCamera::reset()
{
    head_ = 0;

    return true;
}


bool YcbVideoCamera::setFrame(const std::size_t& number)
{
    head_ = number;

    return true;
}


std::pair<bool, cv::Mat> YcbVideoCamera::getRgbImage(const bool& blocking)
{
    std::string file_name = path_rgb_images_ + composeFileName(head_, number_of_digits_) + ".png";

    cv::Mat image = cv::imread(file_name, cv::IMREAD_COLOR);

    if (image.empty())
    {
        std::cout << log_ID_ << "::getRgbImage. Warning: frame " << file_name << " is empty!" << std::endl;
        return std::make_pair(false, cv::Mat());
    }
    cv::resize(image, image, cv::Size(parameters_.width, parameters_.height));

    return std::make_pair(true, image);
}


std::pair<bool, MatrixXf> YcbVideoCamera::getDepthImage(const bool& blocking)
{
    std::FILE* in;
    std::string file_name = path_depth_images_ + composeFileName(head_, number_of_digits_) + ".float";

    if ((in = std::fopen(file_name.c_str(), "rb")) == nullptr)
    {
        std::cout << log_ID_ << "::ctor. Error: cannot load depth frame " + file_name;
        return std::make_pair(true, MatrixXf());
    }

    /* Load image size .*/
    std::size_t dims[2];
    std::fread(dims, sizeof(dims), 1, in);

    /* Load image. */
    float float_image_raw[dims[0] * dims[1]];
    std::fread(float_image_raw, sizeof(float), dims[0] * dims[1], in);

    /* Store image. */
    MatrixXf float_image(dims[1], dims[0]);
    float_image = Map<Matrix<float, -1, -1, RowMajor>>(float_image_raw, dims[1], dims[0]);

    /* Handle resize if required (downscaling from 640x480 to 320x240 is the only one supported). */
    MatrixXf depth;
    if ((float_image.cols() != parameters_.width) && (float_image.rows() != parameters_.height))
    {
        if ((float_image.cols() == 640) && (float_image.rows() == 480) && (parameters_.width == 320) && (parameters_.height == 240))
        {
            depth.resize(parameters_.height, parameters_.width);
            for (std::size_t i = 0; i < float_image.rows(); i += 2)
                for (std::size_t j = 0; j < float_image.cols(); j += 2)
                    depth(i / 2, j / 2) = float_image(i, j);
        }
    }
    else
        depth = float_image;

    std::fclose(in);

    return std::make_pair(true, depth);
}


std::pair<bool, Transform<double, 3, Affine>> YcbVideoCamera::getCameraPose(const bool& blocking)
{
    return std::make_pair(true, Transform<double, 3, Affine>::Identity());
}


std::string YcbVideoCamera::composeFileName(const std::size_t& index, const std::size_t& number_digits)
{
    std::ostringstream ss;
    ss << std::setw(number_digits) << std::setfill('0') << index;
    return ss.str();
}


bool YcbVideoCamera::checkImage(const std::size_t& index)
{
    std::ifstream in;
    std::string file_name = path_rgb_images_ + composeFileName(index, number_of_digits_) + ".png";
    in.open(file_name);

    bool outcome = in.is_open();
    if (!outcome)
    {
        std::cout << log_ID_ << "::checkImage. Warning: cannot open file " << file_name << std::endl;
    }

    in.close();

    return outcome;
}
