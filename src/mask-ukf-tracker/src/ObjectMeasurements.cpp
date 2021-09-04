/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ObjectMeasurements.h>

#include <BayesFilters/Gaussian.h>

#include <mlpack/methods/approx_kfn/drusilla_select.hpp>

#include <armadillo>
#include <vector>

using namespace bfl;
using namespace Eigen;
using namespace mlpack::neighbor;
using MatrixXu = Matrix<std::size_t, Dynamic, Dynamic>;

ObjectMeasurements::ObjectMeasurements
(
    std::unique_ptr<Camera> camera,
    std::shared_ptr<PointCloudSegmentation> segmentation,
    std::shared_ptr<PointCloudPrediction> prediction,
    const Eigen::Ref<const Eigen::Matrix3d>& visual_noise_covariance,
    const std::string& depth_fetch_mode
) :
    camera_(std::move(camera)),
    segmentation_(segmentation),
    prediction_(prediction),
    visual_noise_covariance_(visual_noise_covariance),
    depth_fetch_mode_(depth_fetch_mode)
{ }


ObjectMeasurements::~ObjectMeasurements()
{
    disable_log();
}


std::pair<bool, Data> ObjectMeasurements::measure(const Data& data) const
{
    return std::make_pair(true, measurement_);
}


bool ObjectMeasurements::freeze(const Data& data)
{
    // Freeze camera
    measurements_available_ = camera_->freeze();
    if (!measurements_available_)
        return false;

    // Get depth image
    if(!getDepth())
        return false;

    // Freeze segmentation
    if (!segmentation_->freezeSegmentation(*camera_))
        return false;

    // Get 3D point cloud.
    bool blocking_call = false;
    bool valid_point_cloud;
    MatrixXd point_cloud;
    std::size_t expected_num_points;
    std::tie(valid_point_cloud, expected_num_points, point_cloud) = segmentation_->extractPointCloud(*camera_, depth_, 10.0);
    if (!valid_point_cloud)
        return false;

    // Extract last belief available about object pose
    Gaussian last_belief = any::any_cast<Gaussian>(data);

    if (outlier_rejection_)
    {
        double ratio = double(point_cloud.cols()) / double(expected_num_points) * 100.0;
        if ((point_cloud.cols() == 0) || (ratio < 20.0))
        {
            // If too few measurements than expected, or no measurements at all,
            // use virtual measurements sampled on the last belief as described in 3.6

            Transform<double, 3, Affine> pose;
            pose = Translation<double, 3>(last_belief.mean().head<3>());
            pose.rotate(AngleAxisd(AngleAxisd(last_belief.mean(9), Vector3d::UnitZ()) *
                                   AngleAxisd(last_belief.mean(10), Vector3d::UnitY()) *
                                   AngleAxisd(last_belief.mean(11), Vector3d::UnitX())));
            point_cloud = prediction_->evaluateModel(pose);
        }
        else if (outlier_rejection_)
        {
            // If using real measurements, perform outlier rejection as described in 3.4

            // Temporary storage
            MatrixXd point_cloud_tmp;
            MatrixXd prediction_tmp;
            VectorXi excluded_points = VectorXi::Zero(point_cloud.cols());

            point_cloud_tmp.swap(point_cloud);
            do
            {
                // Move inliers to temporary storage
                if (excluded_points.sum() != 0)
                {
                    point_cloud_tmp.resize(point_cloud.rows(), point_cloud.cols() - excluded_points.sum());
                    for (std::size_t i = 0, j = 0; i < point_cloud.cols(); i++)
                    {
                        if (excluded_points(i) == 0)
                        {
                            point_cloud_tmp.col(j).swap(point_cloud.col(i));
                            j++;
                        }
                    }
                    excluded_points = VectorXi::Zero(point_cloud_tmp.cols());
                }

                // Predict point cloud using current object pose
                MatrixXd prediction_tmp;
                MatrixXd prediction_vector;
                std::tie(std::ignore, prediction_vector) = prediction_->predictPointCloud(last_belief.mean(), Map<MatrixXd>(point_cloud_tmp.data(), point_cloud_tmp.size(), 1));
                prediction_tmp = Map<MatrixXd>(prediction_vector.col(0).data(), 3, prediction_vector.col(0).size() / 3);

                // Initialize DrusillaSelect
                bool use_drusilla = true;
                arma::mat point_cloud_reference = arma::mat(point_cloud_tmp.data(), point_cloud_tmp.rows(), point_cloud_tmp.cols(), false, false);
                std::unique_ptr<DrusillaSelect<>> akfn;
                try
                {
                    std::size_t drusilla_l = 5;
                    std::size_t drusilla_m = 5;
                    akfn = std::unique_ptr<DrusillaSelect<>>(new DrusillaSelect<>(point_cloud_reference, drusilla_l, drusilla_m));
                }
                catch (std::invalid_argument)
                {
                    // If drusilla_l * drusilla_m > point_cloud_tmp.cols() DrusillaSelect cannot be used
                    use_drusilla = false;
                }

                // Search for outliers
                std::size_t k = 5;
                for (std::size_t i = 0; i < point_cloud_tmp.cols(); i++)
                {
                    // Query point
                    Vector3d q = point_cloud_tmp.col(i);

                    MatrixXu neighbors;
                    if (use_drusilla)
                    {
                        arma::Mat<size_t> neighbors_arma;
                        arma::mat distances;

                        arma::mat query_arma(q.data(), 3, 1, false, false);
                        akfn->Search(query_arma, k, neighbors_arma, distances);
                        neighbors = Map<MatrixXu>(neighbors_arma.memptr(), neighbors_arma.n_rows, neighbors_arma.n_cols);
                    }
                    else
                    {
                        // Since DrusillaSelect is not available, a brute force approach with k = 1 is used
                        std::size_t max_index;
                        (point_cloud_tmp.colwise() - q).colwise().norm().maxCoeff(&max_index);
                        neighbors.resize(1, 1);
                        neighbors(0, 0) = max_index;
                    }

                    // Projected point
                    Vector3d q_p = prediction_tmp.col(i);

                    for (std::size_t j = 0; j < neighbors.rows(); j++)
                    {
                        // Furthest point to q
                        Vector3d f = point_cloud_tmp.col(neighbors.col(0)(j));

                        // Projected point
                        Vector3d f_p = prediction_tmp.col(neighbors.col(0)(j));

                        // Distance in real point cloud
                        double dist_real = (q - f).norm();

                        // Distance in projection
                        double dist_proj = (q_p - f_p).norm();

                        if (abs(dist_proj - dist_real) > 0.01)
                        {
                            // Distance of outlier candidates to their projections
                            double dist_1 = (q - q_p).norm();
                            double dist_2 = (f - f_p).norm();
                            if (dist_1 > dist_2)
                            {
                                excluded_points(i) = 1;
                            }
                            else
                            {
                                excluded_points(neighbors.col(0)(j))  = 1;
                            }
                        }
                    }
                }
                point_cloud.swap(point_cloud_tmp);
            } while(excluded_points.sum() != 0);
        }
    }

    // Resize measurements to be a column vector.
    measurement_.resize(3 * point_cloud.cols(), 1);
    measurement_.swap(Map<MatrixXd>(point_cloud.data(), point_cloud.size(), 1));

    // Log measurements
    logger(measurement_.transpose());

    return true;
}


std::pair<std::size_t, std::size_t> ObjectMeasurements::getOutputSize() const
{
    return std::make_pair(measurement_.size(), 0);
}


std::pair<bool, bfl::Data> ObjectMeasurements::predictedMeasure(const Eigen::Ref<const Eigen::MatrixXd>& cur_states) const
{
    bool valid_measure;
    bfl::Data measurement;
    std::tie(valid_measure, measurement) = measure();

    if (!valid_measure)
        return std::make_pair(false, Data());

    bool valid_prediction;
    MatrixXd prediction;
    std::tie(valid_prediction, prediction) = prediction_->predictPointCloud(cur_states, any::any_cast<MatrixXd>(measurement).col(0));

    if (!valid_prediction)
        return std::make_pair(false, Data());

    return std::make_pair(true, prediction);
}


std::pair<bool, bfl::Data> ObjectMeasurements::innovation(const bfl::Data& predicted_measurements, const bfl::Data& measurements) const
{
    MatrixXd innovation = -(any::any_cast<MatrixXd>(predicted_measurements).colwise() - any::any_cast<MatrixXd>(measurements).col(0));

    return std::make_pair(true, std::move(innovation));
}


std::pair<bool, Eigen::MatrixXd> ObjectMeasurements::getNoiseCovarianceMatrix() const
{
    return std::make_pair(true, visual_noise_covariance_);
}


bool ObjectMeasurements::setProperty(const std::string& property)
{
    if (property == "reset")
    {
        reset();

        return true;
    }
    else if (property == "measurements_available")
    {
        return measurements_available_;
    }
    else if (property == "disable_log")
    {
        disable_log();

        return true;
    }

    return false;
}


void ObjectMeasurements::enableOutlierRejection(const bool& enable)
{
    outlier_rejection_ = enable;
}


bool ObjectMeasurements::getDepth()
{
    std::string mode = depth_fetch_mode_;
    if (!depth_initialized_)
    {
        // in case a depth was never received
        // it is required to wait at least for the first image in blocking mode
        mode = "new_image";
    }

    bool valid_depth;
    MatrixXf tmp_depth;
    std::tie(valid_depth, tmp_depth) = camera_->getDepthImage(mode == "new_image");

    if (valid_depth)
    {
        depth_ = std::move(tmp_depth);
        depth_initialized_ = true;
    }

    if (mode == "skip")
        return depth_initialized_ && valid_depth;
    else
        return depth_initialized_;
}


void ObjectMeasurements::reset()
{
    depth_initialized_ = false;

    measurements_available_ = true;

    camera_->reset();

    segmentation_->setProperty("reset");
}
