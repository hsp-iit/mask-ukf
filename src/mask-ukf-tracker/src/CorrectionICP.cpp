/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <CorrectionICP.h>

#include <Eigen/Dense>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>


using namespace bfl;
using namespace Eigen;


CorrectionICP::CorrectionICP(std::unique_ptr<bfl::AdditiveMeasurementModel> measurement_model, const std::string& point_cloud_filename) :
    measurement_model_(std::move(measurement_model))
{
    /* Load model point cloud from file. */
    bool valid_cloud = false;
    std::tie(valid_cloud, model_) = loadPointCloudFromFile(point_cloud_filename);
    if (!valid_cloud)
    {
        std::string err = log_ID_ + "::ctor. Error: cannot load point cloud from file " + point_cloud_filename;
        throw(std::runtime_error(err));
    }
}


CorrectionICP::~CorrectionICP()
{ }


void CorrectionICP::correctStep(const bfl::GaussianMixture& pred_state, bfl::GaussianMixture& corr_state)
{
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    /* Get the current measurement if available. */
    bool valid_measurement;
    Data measurement;
    std::tie(valid_measurement, measurement) = measurement_model_->measure();

    if (!valid_measurement)
    {
        corr_state = pred_state;
        return;
    }
    MatrixXd meas_vector = any::any_cast<MatrixXd&&>(std::move(measurement)).col(0);
    MatrixXd meas = Map<MatrixXd>(meas_vector.data(), 3, meas_vector.size() / 3);

    /* Transform the model in the current pose. */
    VectorXd state = pred_state.mean(0);
    Transform<double, 3, Affine> pose;
    pose = Translation<double, 3>(state.head<3>());
    pose.rotate(AngleAxisd(state(9), Vector3d::UnitZ()) *
                AngleAxisd(state(10), Vector3d::UnitY()) *
                AngleAxisd(state(11), Vector3d::UnitX()));

    MatrixXd model_transformed = pose * model_.topRows<3>().colwise().homogeneous();

    /* Do ICP. */
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZ>);

    /* Fill in source cloud */
    cloud_source->width = meas.size() / 3.0;
    cloud_source->height = 1;
    cloud_source->is_dense = false;
    cloud_source->points.resize (meas.size() / 3.0);
    for (size_t i = 0; i < cloud_source->points.size(); ++i)
    {
        cloud_source->points[i].x = meas.col(i)(0);
        cloud_source->points[i].y = meas.col(i)(1);
        cloud_source->points[i].z = meas.col(i)(2);
    }

    /* Fill in pcl cloud. */
    cloud_target->width = model_transformed.cols();
    cloud_target->height = 1;
    cloud_target->is_dense = false;
    cloud_target->points.resize (model_transformed.cols());
    for (size_t i = 0; i < cloud_target->points.size(); ++i)
    {
        cloud_target->points[i].x = model_transformed.col(i)(0);
        cloud_target->points[i].y = model_transformed.col(i)(1);
        cloud_target->points[i].z = model_transformed.col(i)(2);
    }

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_source);
    icp.setInputTarget(cloud_target);

    // Set the max correspondence distance (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance (0.05);

    pcl::PointCloud<pcl::PointXYZ> aligned;
    icp.align(aligned);

    if (icp.hasConverged())
    {
        Matrix4f transformation = icp.getFinalTransformation();
        Matrix4d transformation_double = transformation.cast<double>();
        Transform<double, 3, Affine> pose_delta(transformation_double);
        pose = pose_delta.inverse() * pose;

        corr_state.mean(0).head<3>() = pose.translation();
        corr_state.mean(0).tail<3>() = pose.rotation().eulerAngles(2, 1, 0);
    }
    else
        // If not convergence, the predicted state is used as corrected state
        // Warning: in the ICP implementation the predicted state corresponds to
        // the corrected state at the previous step since there is no motion model involved
        // (see class StaticPrediction for more details)
        corr_state = pred_state;
}


MeasurementModel& CorrectionICP::getMeasurementModel()
{
    return *measurement_model_;
}


std::pair<bool, MatrixXd> CorrectionICP::loadPointCloudFromFile(const std::string& filename)
{
    MatrixXd data;

    std::ifstream istrm(filename);

    if (!istrm.is_open())
    {
        istrm.close();

        return std::make_pair(false, MatrixXd(0,0));
    }

    std::vector<std::string> istrm_strings;
    std::string line;
    while (std::getline(istrm, line))
        istrm_strings.push_back(line);

    data.resize(3, istrm_strings.size());
    std::size_t found_lines = 0;
    for (auto line : istrm_strings)
    {
        std::size_t found_fields = 0;
        std::string number_str;
        std::istringstream iss(line);

        while (iss >> number_str)
        {
            std::size_t index = (3 * found_lines) + found_fields;
            *(data.data() + index) = std::stod(number_str);
            found_fields++;
        }
        if (found_fields != 3)
            return std::make_pair(false, MatrixXd(0,0));

        found_lines++;
    }

    istrm.close();

    return std::make_pair(true, data);
}
