/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <Camera.h>

#include <iostream>

using namespace Eigen;


Camera::~Camera()
{ }


bool Camera::initialize()
{
    bool ok = true;

    // Cache the deprojection matrix once for all
    ok &= evalDeprojectionMatrix();

    return ok;
}


bool Camera::freeze()
{
    return true;
}


bool Camera::reset()
{
    return false;
}


bool Camera::setFrame(const std::size_t& number)
{

    return false;
}


std::pair<bool, MatrixXd> Camera::getDeprojectionMatrix()
{
    if (!deprojection_matrix_initialized_)
        return std::make_pair(false, MatrixXd());

    return std::make_pair(true, deprojection_matrix_);
}


bool Camera::evalDeprojectionMatrix()
{
    if (!parameters_.initialized)
    {
        std::cout << log_ID_base_ << "::evalDeprojectionMatrix. Camera parameters not initialized. Did you initialize the class member 'parameters_' in the derived class?";

        return false;
    }

    // Allocate storage
    deprojection_matrix_.resize(3, parameters_.width * parameters_.height);

    // Evaluate deprojection matrix
    int i = 0;
    for (std::size_t u = 0; u < parameters_.width; u++)
    {
        for (std::size_t v = 0; v < parameters_.height; v++)
        {
            deprojection_matrix_(0, i) = (u - parameters_.cx) / parameters_.fx;
            deprojection_matrix_(1, i) = (v - parameters_.cy) / parameters_.fy;
            deprojection_matrix_(2, i) = 1.0;

            i++;
        }
    }

    deprojection_matrix_initialized_ = true;

    return true;
}


std::pair<bool, CameraParameters> Camera::getIntrinsicParameters() const
{
    if (!parameters_.initialized)
        return std::make_pair(false, CameraParameters());

    return std::make_pair(true, parameters_);
}
