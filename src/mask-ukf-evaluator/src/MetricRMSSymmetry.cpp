/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <MetricRMSSymmetry.h>
#include <limits>

using namespace Eigen;


MetricRMSSymmetry::MetricRMSSymmetry(const Eigen::VectorXd& symmetry_axis) :
    symmetry_axis_(symmetry_axis)
{ }


MetricRMSSymmetry::~MetricRMSSymmetry()
{ }


VectorXd MetricRMSSymmetry::evaluate(const Eigen::Transform<double, 3, Eigen::Affine>& estimate, const Eigen::Transform<double, 3, Eigen::Affine>& ground_truth, const Eigen::MatrixXd& model)
{
    VectorXd errors(2);

    if (((estimate.translation()(0) == std::numeric_limits<double>::infinity()) &&
         (estimate.translation()(1) == std::numeric_limits<double>::infinity()) &&
         (estimate.translation()(2) == std::numeric_limits<double>::infinity())) ||
        ((estimate.translation()(0) == 0.0) &&
         (estimate.translation()(1) == 0.0) &&
         (estimate.translation()(2) == 0.0)))
    {
        // Warning: this check is required since some DenseFusion frames might be missing.
        // When missing, they are identified as having an infinite estimate or all zeros
        // In such cases, we decided to assign a null error for the evaluation of the RMSE
        errors(0) = 0.0;
        errors(1) = 0.0;

        return errors;
    }

    errors(0) = (estimate.translation() - ground_truth.translation()).squaredNorm();

    // Use twist-swing decomposition to extract the relevant rotation taking into account the symmetry axis
    AngleAxisd twist;
    AngleAxisd swing;
    std::tie(twist, swing) = twistSwingDecomposition(AngleAxisd((estimate.rotation().transpose() * ground_truth.rotation())), symmetry_axis_);
    errors(1) = std::pow(swing.angle(), 2);

    return errors;
}


std::pair<Eigen::AngleAxisd, Eigen::AngleAxisd> MetricRMSSymmetry::twistSwingDecomposition(const Eigen::AngleAxisd& rotation, const Vector3d& twist_axis)
{
    // Find twist-swing decomposition according to
    // https://www.gamedev.net/forums/topic/696882-swing-twist-interpolation-sterp-an-alternative-to-slerp/

    // Convert rotation to a quaternion
    Quaterniond q(rotation);

    // Resulting rotations
    AngleAxisd swing;
    AngleAxisd twist;

    // Handle singularity (rotation by 180 degree)
    Vector3d q_vector(q.x(), q.y(), q.z());
    if (q_vector.squaredNorm() < std::numeric_limits<double>::min())
    {
        // Vector3 rotatedTwistAxis = q * twistAxis;
        Vector3d rotated_twist_axis = q.toRotationMatrix() * twist_axis;
        Vector3d swing_axis = twist_axis.cross(rotated_twist_axis);

        if (swing_axis.squaredNorm() > std::numeric_limits<double>::min())
        {
            double swing_angle = std::acos(twist_axis.normalized().dot(swing_axis.normalized()));
            swing = AngleAxisd(swing_angle, swing_axis);
        }
        else
        {
            // More singularity: rotation axis parallel to twist axis
            swing = AngleAxisd(Transform<double, 3, Affine>::Identity().rotation());
        }

        // always twist 180 degree on singularity
        twist = AngleAxisd(M_PI, twist_axis);

        return std::make_pair(twist, swing);
    }

    // General case
    Vector3d projection = twist_axis * q_vector.dot(twist_axis);
    Quaterniond twist_q(q.w(), projection(0), projection(1), projection(2));
    twist_q.normalize();
    Quaterniond swing_q = q * twist_q.inverse();

    return std::make_pair(AngleAxisd(twist_q), AngleAxisd(swing_q));
}
