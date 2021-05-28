/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <Camera.h>
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

#include <yarp/os/Bottle.h>
#include <yarp/os/ResourceFinder.h>

using namespace bfl;
using namespace yarp::os;
using namespace Eigen;


VectorXd load_vector_double(Bottle &rf, const std::string key, const std::size_t size);


std::string eigen_to_string(const Ref<const VectorXd>& v);


int main(int argc, char** argv)
{
    const std::string log_ID = "[Main]";
    std::cout << log_ID << "Configuring and starting module..." << std::endl;

    ResourceFinder rf;
    rf.setVerbose();
    rf.setDefaultContext("object-tracking");
    rf.configure(argc, argv);

    /* Get algorithm name. */
    const std::string algorithm = rf.check("algorithm", Value("UKF")).asString();

    /* Get autostart flag. */
    const bool autostart = rf.check("autostart", Value(false)).asBool();

    /* Get initial conditions. */
    Bottle rf_initial_conditions = rf.findGroup("INITIAL_CONDITION");
    VectorXd cov_x_0             = load_vector_double(rf_initial_conditions, "cov_x_0",       3);
    VectorXd cov_v_0             = load_vector_double(rf_initial_conditions, "cov_v_0",       3);
    VectorXd cov_eul_0           = load_vector_double(rf_initial_conditions, "cov_eul_0",     3);
    VectorXd cov_eul_dot_0       = load_vector_double(rf_initial_conditions, "cov_eul_dot_0", 3);

    /* Camera parameters. */
    Bottle rf_camera = rf.findGroup("CAMERA");
    const std::string camera_name         = rf_camera.check("name", Value("")).asString();

    /* Kinematic model. */
    Bottle rf_kinematic_model = rf.findGroup("KINEMATIC_MODEL");
    VectorXd kin_q_x             = load_vector_double(rf_kinematic_model, "q_x", 3);
    VectorXd kin_q_eul           = load_vector_double(rf_kinematic_model, "q_eul", 3);
    const double kin_rate        = rf_kinematic_model.check("rate", Value(30.0)).asDouble();
    const bool kin_est_period    = rf_kinematic_model.check("estimate_period", Value(true)).asBool();

    /* Measurement model. */
    Bottle rf_measurement_model = rf.findGroup("MEASUREMENT_MODEL");
    VectorXd visual_covariance           = load_vector_double(rf_measurement_model, "visual_covariance", 3);
    MatrixXd visual_covariance_diagonal  = visual_covariance.asDiagonal();

    /* Unscented transform. */
    Bottle rf_unscented_transform = rf.findGroup("UNSCENTED_TRANSFORM");
    const double ut_alpha = rf_unscented_transform.check("alpha", Value("1.0")).asDouble();
    const double ut_beta  = rf_unscented_transform.check("beta", Value("2.0")).asDouble();
    const double ut_kappa = rf_unscented_transform.check("kappa", Value("0.0")).asDouble();

    /* Point cloud filtering. */
    Bottle rf_point_cloud_filtering = rf.findGroup("POINT_CLOUD_FILTERING");
    const bool pc_outlier_rejection = rf_point_cloud_filtering.check("outlier_rejection", Value(false)).asBool();

    /* Depth. */
    Bottle rf_depth = rf.findGroup("DEPTH");
    const std::string depth_fetch_mode = rf_depth.check("fetch_mode", Value("new_image")).toString();
    const std::size_t depth_stride     = rf_depth.check("stride", Value(1)).asInt();

    /* Mesh parameters. */
    Bottle rf_object = rf.findGroup("OBJECT");
    const std::string object_name      = rf_object.check("object_name", Value("002_master_chef_can")).asString();
    const std::string object_point_cloud_path = "./models/" + object_name;
    const std::string object_data_path        = rf_object.check("path", Value("null")).asString();

    /* Segmentation parameters. */
    Bottle rf_segmentation    = rf.findGroup("SEGMENTATION");
    const std::string segmentation_set  = rf_segmentation.check("masks_set", Value("gt")).asString();
    const bool segmentation_enforce_fps = rf_segmentation.check("enforce_fps", Value(false)).asBool();
    const double segmentation_fps = rf_segmentation.check("fps", Value(5.0)).asDouble();

    /* Logging parameters. */
    Bottle rf_logging = rf.findGroup("LOG");
    bool enable_log = rf_logging.check("enable_log", Value(false)).asBool();
    const std::string log_path = rf_logging.check("absolute_log_path", Value("")).asString();
    if (enable_log && log_path == "")
    {
        std::cout << "Invalid log path. Disabling log..." << std::endl;
        enable_log = false;
    }

    /* Log parameters. */

    std::cout << log_ID << "Algorithm:" << algorithm << std::endl;

    std::cout << log_ID << "Initial conditions:" << std::endl;
    std::cout << log_ID << "- cov_x_0: "          << eigen_to_string(cov_x_0) << std::endl;
    std::cout << log_ID << "- cov_v_0: "          << eigen_to_string(cov_v_0) << std::endl;
    std::cout << log_ID << "- cov_eul_0: "        << eigen_to_string(cov_eul_0) << std::endl;
    std::cout << log_ID << "- cov_eul_dot_0: "    << eigen_to_string(cov_eul_dot_0) << std::endl;

    std::cout << log_ID << "Camera:" << std::endl;
    std::cout << log_ID << "- name:"         << camera_name << std::endl;

    std::cout << log_ID << "Kinematic model:" << std::endl;
    std::cout << log_ID << "- q_x:"             << eigen_to_string(kin_q_x) << std::endl;
    std::cout << log_ID << "- q_eul:"           << eigen_to_string(kin_q_eul) << std::endl;
    std::cout << log_ID << "- rate:"            << kin_rate << std::endl;
    std::cout << log_ID << "- estimate_period:" << kin_est_period << std::endl;

    std::cout << log_ID << "Measurement model:" << std::endl;
    std::cout << log_ID << "- visual_covariance:" << eigen_to_string(visual_covariance) << std::endl;

    std::cout << log_ID << "Unscented transform:" << std::endl;
    std::cout << log_ID << "- alpha:" << ut_alpha << std::endl;
    std::cout << log_ID << "- beta:"  << ut_beta << std::endl;
    std::cout << log_ID << "- kappa:" << ut_kappa << std::endl;

    std::cout << log_ID << "Point cloud filtering:" << std::endl;
    std::cout << log_ID << "- outlier_rejection:" << pc_outlier_rejection << std::endl;

    std::cout << log_ID << "Depth:" << std::endl;
    std::cout << log_ID << "- fetch_mode:" << depth_fetch_mode << std::endl;
    std::cout << log_ID << "- stride:" << depth_stride << std::endl;

    std::cout << log_ID << "Object:" << std::endl;
    std::cout << log_ID << "- object_name:"         << object_name << std::endl;
    std::cout << log_ID << "- object_data_path:"    << object_data_path << std::endl;
    std::cout << log_ID << "- point cloud path is:" << object_point_cloud_path << std::endl;

    std::cout << log_ID << "Segmentation:" << std::endl;
    std::cout << log_ID << "- masks_set:" << segmentation_set << std::endl;
    std::cout << log_ID << "- enforce_fps:" << segmentation_enforce_fps << std::endl;
    if (segmentation_enforce_fps)
        std::cout << log_ID << "- fps:" << segmentation_fps << std::endl;

    std::cout << log_ID << "Logging:" << std::endl;
    std::cout << log_ID << "- enable_log:"        << enable_log << std::endl;
    std::cout << log_ID << "- absolute_log_path:" << log_path << std::endl;

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


VectorXd load_vector_double(Bottle &rf, const std::string key, const std::size_t size)
{
    bool ok = true;

    if (rf.find(key).isNull())
        ok = false;

    Bottle* b = rf.find(key).asList();
    if (b == nullptr)
        ok = false;

    if (b->size() != size)
        ok = false;

    if (!ok)
    {
        std::cerr << "[Main]" << "Unable to load vector" << key << std::endl;
        std::exit(EXIT_FAILURE);
    }

    VectorXd vector(size);
    for (std::size_t i = 0; i < b->size(); i++)
    {
        Value item_v = b->get(i);
        if (item_v.isNull())
            return VectorXd(0);

        if (!item_v.isDouble())
            return VectorXd(0);

        vector(i) = item_v.asDouble();
    }

    return vector;
}


std::string eigen_to_string(const Ref<const VectorXd>& v)
{
    std::stringstream ss;
    ss << v.transpose();
    return ss.str();
}
