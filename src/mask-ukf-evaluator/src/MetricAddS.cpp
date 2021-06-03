/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <MetricAddS.h>
#include <PointCloudAdaptor.h>
#include <kdTree.h>

#include <memory>

using namespace Eigen;
using namespace nanoflann;


MetricAddS::MetricAddS()
{ }


MetricAddS::~MetricAddS()
{ }


VectorXd MetricAddS::evaluate(const Eigen::Transform<double, 3, Eigen::Affine>& estimate, const Eigen::Transform<double, 3, Eigen::Affine>& ground_truth, const Eigen::MatrixXd& model)
{
    VectorXd values(1);

    if (((estimate.translation()(0) == std::numeric_limits<double>::infinity()) &&
         (estimate.translation()(1) == std::numeric_limits<double>::infinity()) &&
         (estimate.translation()(2) == std::numeric_limits<double>::infinity())) ||
        ((estimate.translation()(0) == 0.0) &&
         (estimate.translation()(1) == 0.0) &&
         (estimate.translation()(2) == 0.0)))
    {
        // Warning: this check is required since some DenseFusion frames might be missing.
        // When missing, they are identified as having an infinite estimate or all zeros
        // In such cases, they are assigned with an infinite error as per the code
        // that can be found here https://github.com/j96w/DenseFusion/blob/master/replace_ycb_toolbox/evaluate_poses_keyframe.m
        values(0) = std::numeric_limits<double>::infinity();

        return values;
    }

    /* Evaluate model in estimated pose. */
    MatrixXd estimated_model = estimate * model.colwise().homogeneous();

    /* Evaluate model in ground truth pose. */
    MatrixXd ground_truth_model = ground_truth * model.colwise().homogeneous();

    /* Evaluate ADD-S metric. */
    MatrixXd estimated_model_symmetric(estimated_model.rows(), estimated_model.cols());

    std::unique_ptr<PointCloudAdaptor> adapted_cloud = std::unique_ptr<PointCloudAdaptor>(new PointCloudAdaptor(estimated_model));
    std::unique_ptr<kdTree> tree = std::unique_ptr<kdTree>(new kdTree(3 /* dim */, *adapted_cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */)));
    tree->buildIndex();

    /* Evaluate closest points using a kdtree. */
    for (std::size_t i = 0; i < ground_truth_model.cols(); i++)
    {
        const Ref<const Vector3d> gt_i = ground_truth_model.col(i);

        std::size_t ret_index;
        double out_dist_sqr;
        KNNResultSet<double> resultSet(1);
        resultSet.init(&ret_index, &out_dist_sqr);
        tree->findNeighbors(resultSet, gt_i.data(), nanoflann::SearchParams(10));

        estimated_model_symmetric.col(i) = estimated_model.col(ret_index);
    }

    MatrixXd difference = ground_truth_model - estimated_model_symmetric;

    values(0) = difference.colwise().norm().sum() / difference.cols();

    return values;
}
