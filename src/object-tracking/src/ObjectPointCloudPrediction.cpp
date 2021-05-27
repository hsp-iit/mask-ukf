/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ObjectPointCloudPrediction.h>

#include <fstream>

using namespace Eigen;
using namespace nanoflann;


ObjectPointCloudPrediction::ObjectPointCloudPrediction(const std::string& point_cloud_filename)
{
    // Load the point cloud from file
    bool valid_cloud = false;
    std::tie(valid_cloud, cloud_) = loadPointCloudFromFile(point_cloud_filename);
    if (!valid_cloud)
    {
        std::string err = log_ID_ + "::ctor. Error: cannot load point cloud from file " + point_cloud_filename;
        throw(std::runtime_error(err));
    }
}


ObjectPointCloudPrediction::~ObjectPointCloudPrediction()
{ }


std::pair<bool, MatrixXd> ObjectPointCloudPrediction::predictPointCloud(ConstMatrixXdRef state, ConstVectorXdRef meas)
{
    // Check if meas size is multiple of 3
    if ((meas.size() % 3) != 0)
        return std::make_pair(false, MatrixXd(0, 0));

    // Reshape measurement as a matrix
    Map<const MatrixXd> meas_matrix(meas.data(), 3, meas.size() / 3);

    // Move sampled point cloud to robot frame according to states stored in state
    std::vector<MatrixXd> clouds(state.cols());
    std::vector<std::unique_ptr<PointCloudAdaptor>> adapted_clouds(state.cols());
    std::vector<std::unique_ptr<kdTree>> trees(state.cols());

    for (std::size_t i = 0; i < state.cols(); i++)
    {
        // Compose translation
        Transform<double, 3, Eigen::Affine> pose;
        pose = Translation<double, 3>(state.col(i).segment(0, 3));

        // Compose rotation
        AngleAxisd rotation(AngleAxis<double>(state(9,  i), Vector3d::UnitZ()) *
                            AngleAxis<double>(state(10, i), Vector3d::UnitY()) *
                            AngleAxis<double>(state(11, i), Vector3d::UnitX()));
        pose.rotate(rotation);

        // Express point cloud sampled on the model in robot frame
        clouds[i] = pose * cloud_.topRows<3>().colwise().homogeneous();

        // Initialize tree
        adapted_clouds[i] = std::unique_ptr<PointCloudAdaptor>(new PointCloudAdaptor(clouds.at(i)));
        trees[i] = std::unique_ptr<kdTree>(new kdTree(3 /* dim */, *adapted_clouds.at(i), KDTreeSingleIndexAdaptorParams(10 /* max leaf */)));
        trees[i]->buildIndex();
    }

    // Predict measurement
    MatrixXd predictions(meas.size(), state.cols());

    for (std::size_t i = 0; i < state.cols(); i++)
    {
        for (std::size_t j = 0; j < meas.size() / 3; j++)
        {
            const Ref<const Vector3d> meas_j = meas_matrix.col(j);

            std::size_t ret_index;
            double out_dist_sqr;
            KNNResultSet<double> resultSet(1);
            resultSet.init(&ret_index, &out_dist_sqr);
            // Querying tree_ is thread safe as per this issue
            // https://github.com/jlblancoc/nanoflann/issues/54
            trees.at(i)->findNeighbors(resultSet, meas_j.data(), nanoflann::SearchParams(10));

            predictions.col(i).segment<3>(j * 3) = clouds.at(i).col(ret_index);
        }
    }

    return std::make_pair(true, predictions);
}


std::pair<bool, MatrixXd> ObjectPointCloudPrediction::evaluateDistances(ConstMatrixXdRef state, ConstVectorXdRef meas)
{
    // Check if meas size is multiple of 3
    if ((meas.size() % 3) != 0)
        return std::make_pair(false, MatrixXd(0, 0));

    MatrixXd squared_distances(state.cols(), meas.size() / 3);

    // Reshape measurement as a matrix
    Map<const MatrixXd> meas_matrix(meas.data(), 3, meas.size() / 3);

    // Move sampled point cloud to robot frame according to states stored in state
    std::vector<MatrixXd> clouds(state.cols());
    std::vector<std::unique_ptr<PointCloudAdaptor>> adapted_clouds(state.cols());
    std::vector<std::unique_ptr<kdTree>> trees(state.cols());

    for (std::size_t i = 0; i < state.cols(); i++)
    {
        // Compose translation
        Transform<double, 3, Eigen::Affine> pose;
        pose = Translation<double, 3>(state.col(i).segment(0, 3));

        // Compose rotation
        AngleAxisd rotation(AngleAxis<double>(state(9,  i), Vector3d::UnitZ()) *
                            AngleAxis<double>(state(10, i), Vector3d::UnitY()) *
                            AngleAxis<double>(state(11, i), Vector3d::UnitX()));
        pose.rotate(rotation);

        // Express point cloud sampled on the model in robot frame
        clouds[i] = pose * cloud_.topRows<3>().colwise().homogeneous();

        // Initialize tree
        adapted_clouds[i] = std::unique_ptr<PointCloudAdaptor>(new PointCloudAdaptor(clouds.at(i)));
        trees[i] = std::unique_ptr<kdTree>(new kdTree(3 /* dim */, *adapted_clouds.at(i), KDTreeSingleIndexAdaptorParams(10 /* max leaf */)));
        trees[i]->buildIndex();
    }

    // Eval distances

    for (std::size_t i = 0; i < state.cols(); i++)
    {
        for (std::size_t j = 0; j < meas.size() / 3; j++)
        {
            const Ref<const Vector3d> meas_j = meas_matrix.col(j);

            std::size_t ret_index;
            double out_dist_sqr;
            KNNResultSet<double> resultSet(1);
            resultSet.init(&ret_index, &out_dist_sqr);
            // Querying tree_ is thread safe as per this issue
            // https://github.com/jlblancoc/nanoflann/issues/54
            trees.at(i)->findNeighbors(resultSet, meas_j.data(), nanoflann::SearchParams(10));

            squared_distances(i, j) = out_dist_sqr;
        }
    }

    return std::make_pair(true, squared_distances);
}


Eigen::MatrixXd ObjectPointCloudPrediction::evaluateModel(const Transform<double, 3, Affine>& object_pose)
{
    return object_pose * cloud_.colwise().homogeneous();
}


std::pair<bool, MatrixXd> ObjectPointCloudPrediction::loadPointCloudFromFile(const std::string& filename)
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
