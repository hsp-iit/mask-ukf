/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <Camera.h>
#include <ConfigParser.h>
#include <Correction.h>
#include <CorrectionICP.h>
#include <DiscretizedKinematicModel.h>
#include <InitGroundTruth.h>
#include <MaskSegmentation.h>
#include <ObjectPointCloudPrediction.h>
#include <PointCloudSegmentation.h>
#include <StaticPrediction.h>
#include <UKFFilter.h>
#include <YCBVideoCamera.h>

#include <BayesFilters/AdditiveMeasurementModel.h>
#include <BayesFilters/FilteringAlgorithm.h>
#include <BayesFilters/GaussianCorrection.h>
#include <BayesFilters/GaussianPrediction.h>
#include <BayesFilters/KFPrediction.h>

#include <Eigen/Dense>

#include <cstdlib>
#include <string>
#include <sstream>

using namespace bfl;
using namespace Eigen;


int main(int argc, char** argv)
{
    const std::string log_ID = "[Main] ";
    std::cout << log_ID << "Configuring and starting module..." << std::endl;

    ConfigParser conf(argc, argv);

    /* Get algorithm name. */
    std::string algorithm; conf("algorithm", algorithm);

    /* Get autostart flag. */
    bool autostart; conf("autostart", autostart);

    /* Get initial conditions. */
    VectorXd cov_x_0; conf("initial_condition::cov_x_0", cov_x_0);
    VectorXd cov_v_0; conf("initial_condition::cov_v_0", cov_v_0);
    VectorXd cov_eul_0; conf("initial_condition::cov_eul_0", cov_eul_0);
    VectorXd cov_eul_dot_0; conf("initial_condition::cov_eul_dot_0", cov_eul_dot_0);

    /* Camera parameters. */
    std::string camera_name; conf("camera::name", camera_name);

    /* Kinematic model. */
    VectorXd kin_q_x; conf("kinematic_model::q_x", kin_q_x);
    VectorXd kin_q_eul; conf("kinematic_model::q_eul", kin_q_eul);
    double kin_rate; conf("kinematic_model::rate", kin_rate);
    bool kin_est_period; conf("kinematic_model::estimate_period", kin_est_period);

    /* Measurement model. */
    VectorXd visual_covariance; conf("measurement_model::visual_covariance", visual_covariance);
    MatrixXd visual_covariance_diagonal = visual_covariance.asDiagonal();

    /* Unscented transform. */
    double ut_alpha; conf("unscented_transform::alpha", ut_alpha);
    double ut_beta; conf("unscented_transform::beta", ut_beta);
    double ut_kappa; conf("unscented_transform::kappa", ut_kappa);

    /* Point cloud filtering. */
    bool pc_outlier_rejection; conf("point_cloud_filtering::outlier_rejection", pc_outlier_rejection);

    /* Depth. */
    std::string depth_fetch_mode; conf("depth::fetch_mode", depth_fetch_mode);
    int depth_stride; conf("depth::stride", depth_stride);

    /* Mesh parameters. */
    std::string object_name; conf("object::object_name", object_name);
    std::string object_point_cloud_path = "./models/" + object_name;
    std::string object_data_path; conf("object::path", object_data_path);

    /* Segmentation parameters. */
    std::string segmentation_set; conf("segmentation::masks_set", segmentation_set);
    bool segmentation_enforce_fps; conf("segmentation::enforce_fps", segmentation_enforce_fps);
    double segmentation_fps; conf("segmentation::fps", segmentation_fps);

    /* Logging parameters. */
    bool enable_log; conf("log::enable_log", enable_log);
    std::string log_path; conf("log::absolute_log_path", log_path);
    if (enable_log && log_path == "")
    {
        std::cout << "Invalid log path. Disabling log..." << std::endl;
        enable_log = false;
    }

    /* Log parameters. */
    auto eigen_to_string = [](const Ref<const VectorXd>& v)
    {
        std::stringstream ss;
        ss << v.transpose();
        return ss.str();
    };

    std::cout << log_ID << "Algorithm:" << algorithm << std::endl;

    std::cout << log_ID << "Initial conditions:" << std::endl;
    std::cout << log_ID << "- cov_x_0: "          << eigen_to_string(cov_x_0) << std::endl;
    std::cout << log_ID << "- cov_v_0: "          << eigen_to_string(cov_v_0) << std::endl;
    std::cout << log_ID << "- cov_eul_0: "        << eigen_to_string(cov_eul_0) << std::endl;
    std::cout << log_ID << "- cov_eul_dot_0: "    << eigen_to_string(cov_eul_dot_0) << std::endl;

    std::cout << log_ID << "Camera:" << std::endl;
    std::cout << log_ID << "- name: "         << camera_name << std::endl;

    std::cout << log_ID << "Kinematic model:" << std::endl;
    std::cout << log_ID << "- q_x: "             << eigen_to_string(kin_q_x) << std::endl;
    std::cout << log_ID << "- q_eul: "           << eigen_to_string(kin_q_eul) << std::endl;
    std::cout << log_ID << "- rate: "            << kin_rate << std::endl;
    std::cout << log_ID << "- estimate_period: " << kin_est_period << std::endl;

    std::cout << log_ID << "Measurement model:" << std::endl;
    std::cout << log_ID << "- visual_covariance: " << eigen_to_string(visual_covariance) << std::endl;

    std::cout << log_ID << "Unscented transform:" << std::endl;
    std::cout << log_ID << "- alpha: " << ut_alpha << std::endl;
    std::cout << log_ID << "- beta: "  << ut_beta << std::endl;
    std::cout << log_ID << "- kappa: " << ut_kappa << std::endl;

    std::cout << log_ID << "Point cloud filtering:" << std::endl;
    std::cout << log_ID << "- outlier_rejection: " << pc_outlier_rejection << std::endl;

    std::cout << log_ID << "Depth:" << std::endl;
    std::cout << log_ID << "- fetch_mode: " << depth_fetch_mode << std::endl;
    std::cout << log_ID << "- stride: " << depth_stride << std::endl;

    std::cout << log_ID << "Object:" << std::endl;
    std::cout << log_ID << "- object_name: "         << object_name << std::endl;
    std::cout << log_ID << "- object_data_path: "    << object_data_path << std::endl;
    std::cout << log_ID << "- point cloud path is: " << object_point_cloud_path << std::endl;

    std::cout << log_ID << "Segmentation:" << std::endl;
    std::cout << log_ID << "- masks_set: " << segmentation_set << std::endl;
    std::cout << log_ID << "- enforce_fps: " << segmentation_enforce_fps << std::endl;
    if (segmentation_enforce_fps)
        std::cout << log_ID << "- fps: " << segmentation_fps << std::endl;

    std::cout << log_ID << "Logging:" << std::endl;
    std::cout << log_ID << "- enable_log: "        << enable_log << std::endl;
    std::cout << log_ID << "- absolute_log_path: " << log_path << std::endl;

    /**
     * Initialize camera
     */
    std::unique_ptr<Camera> camera;
    if (camera_name == "YCBVideoCamera")
    {
        camera = std::unique_ptr<YcbVideoCamera>
        (
            new YcbVideoCamera(object_data_path)
        );
    }
    else
    {
        std::cerr << log_ID << "The requested camera is not available. Requested camera is" << camera_name << std::endl;;
        std::exit(EXIT_FAILURE);
    }
    camera->initialize();

    /**
     * Initialize object segmentation.
     */
    std::unique_ptr<PointCloudSegmentation> segmentation;
    if (segmentation_enforce_fps)
    {
        segmentation = std::unique_ptr<MaskSegmentation>
        (
            new MaskSegmentation(object_data_path, object_name, depth_stride, segmentation_set, segmentation_fps)
        );
    }
    else
    {
        segmentation = std::unique_ptr<MaskSegmentation>
        (
            new MaskSegmentation(object_data_path, object_name, depth_stride, segmentation_set)
        );
    }

    /**
     * Initialize point cloud prediction.
     */
    std::shared_ptr<PointCloudPrediction> point_cloud_prediction = std::make_shared<ObjectPointCloudPrediction>(object_point_cloud_path);

    /**
     * Initialize measurement model.
     */
    std::unique_ptr<AdditiveMeasurementModel> measurement_model;
    std::unique_ptr<ObjectMeasurements> obj_meas = std::unique_ptr<ObjectMeasurements>
    (
        new ObjectMeasurements(std::move(camera), std::move(segmentation), point_cloud_prediction, visual_covariance_diagonal, depth_fetch_mode)
    );
    obj_meas->enableOutlierRejection(pc_outlier_rejection);
    // if (enable_log)
    //     obj_meas->enable_log(log_path, "object-tracking");
    measurement_model = std::move(obj_meas);

    /**
     * Filter construction.
     */

    /**
     * StateModel
     */
    std::unique_ptr<LinearStateModel> kinematic_model = std::unique_ptr<DiscretizedKinematicModel>
    (
        new DiscretizedKinematicModel(kin_q_x(0), kin_q_x(1), kin_q_x(2), kin_q_eul(0), kin_q_eul(1), kin_q_eul(2), 1.0 / kin_rate, kin_est_period)
    );

    std::size_t dim_linear;
    std::size_t dim_circular;
    std::tie(dim_linear, dim_circular) = kinematic_model->getOutputSize();
    std::size_t state_size = dim_linear + dim_circular;

    /**
     * Initial condition.
     */

    VectorXd initial_covariance(12);
    initial_covariance.head<3>() = cov_x_0;
    initial_covariance.segment<3>(3) = cov_v_0;
    initial_covariance.segment<3>(6) = cov_eul_dot_0;
    initial_covariance.tail<3>() = cov_eul_0;
    std::unique_ptr<InitGroundTruth> initialization = std::unique_ptr<InitGroundTruth>
    (
        new InitGroundTruth(object_data_path, object_name, 0, initial_covariance.asDiagonal())
    );

    /**
     * Prediction step.
     */
    std::unique_ptr<GaussianPrediction> prediction;
    if (algorithm == "UKF")
    {
        prediction = std::unique_ptr<KFPrediction>
        (
            new KFPrediction(std::move(kinematic_model))
        );
    }
    else
    {
        // Used for ICP
        prediction = std::unique_ptr<StaticPrediction>
        (
            new StaticPrediction()
        );
    }

    /**
     * Correction step.
     */

    std::unique_ptr<GaussianCorrection> correction;
    if (algorithm == "UKF")
    {
        // A measurement is made of 3 * N scalars, N being the number of points
        std::size_t measurement_sub_size = 3;
        correction = std::unique_ptr<Correction>
        (
            new Correction(std::move(measurement_model), state_size, ut_alpha, ut_beta, ut_kappa, measurement_sub_size)
        );
    }
    else
    {
        correction = std::unique_ptr<CorrectionICP>
        (
            new CorrectionICP(std::move(measurement_model), object_point_cloud_path)
        );
    }

    /**
     * Filter.
     */
    std::cout << "Initializing filter..." << std::flush;

    std::unique_ptr<UKFFilter> filter = std::unique_ptr<UKFFilter>
    (
        new UKFFilter(std::move(initialization), std::move(prediction), std::move(correction))
    );
    if (enable_log)
        filter->enable_log(log_path, "object-tracking");

    std::cout << "done." << std::endl;

    std::cout << "Booting filter..." << std::flush;

    filter->boot();

    std::cout << "done." << std::endl;

    std::cout << "Running filter..." << std::endl;

    if (autostart)
        filter->run();

    if (!filter->wait())
        return EXIT_FAILURE;

    std::cout << log_ID << "Application closed succesfully.";

    return EXIT_SUCCESS;
}
