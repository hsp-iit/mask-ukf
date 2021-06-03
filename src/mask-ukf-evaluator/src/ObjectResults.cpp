/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <MetricAddS.h>
#include <MetricEmpty.h>
#include <MetricRMS.h>
#include <MetricRMSSymmetry.h>
#include <ObjectResults.h>

#include <fstream>
#include <iostream>
#include <iomanip>
#include <fstream>

using namespace Eigen;


ObjectResults::ObjectResults
(
    const std::string& object_name,
    const bool& object_has_symmetry,
    const Eigen::Vector3d& object_symmetry_axis,
    const std::string& model_path,
    const std::string& result_path,
    const std::vector<std::string>& test_set,
    const std::unordered_map<std::string, std::vector<std::size_t>>& keyframes,
    const std::unordered_map<std::string, std::pair<std::size_t, std::size_t>>& excluded_keyframes,
    const std::vector<std::string>& metrics,
    const double& max_error
)
{
    MatrixXd model;
    bool valid_model = false;
    std::tie(valid_model, model) = loadPointCloudFromFile(model_path + "/" + object_name);
    if (!valid_model)
        throw(std::runtime_error("[main] Cannot load model of object " + object_name));

    for (auto name : metrics)
    {
        std::string metric_name = name;
        if ((metric_name == "RMSSymmetry") && (!object_has_symmetry))
            metric_name = "RMS";

        if (metric_name == "ADD-S")
            metric_.emplace(metric_name, std::unique_ptr<MetricAddS>(new MetricAddS()));
        else if (metric_name == "RMS")
            metric_.emplace(metric_name, std::unique_ptr<MetricRMS>(new MetricRMS()));
        else if (metric_name == "RMSSymmetry")
            metric_.emplace(metric_name, std::unique_ptr<MetricRMSSymmetry>(new MetricRMSSymmetry(object_symmetry_axis)));
        else if (metric_name == "FPS")
            metric_.emplace(metric_name, std::unique_ptr<MetricEmpty>(new MetricEmpty()));
        else
            throw(std::runtime_error(log_ID_ + "::ctor. Error: cannot load requested metric " + name + "."));
    }

    for (auto video_name : test_set)
        results_.emplace(video_name, SingleVideoResult(object_name, video_name, result_path));

    std::cout << log_ID_ << "::ctor. Object name: " << object_name << std::endl;
    for (auto& metric_item : metric_)
    {
        counter_ = 0;
        std::size_t counter_good = 0;
        rmse_position_ = 0;
        rmse_orientation_ = 0;
        fps_ = 0;

        std::unordered_map<std::string, double> partials_ratio;
        std::unordered_map<std::string, double> partials_auc;
        std::unordered_map<std::string, double> partials_rmse_position;
        std::unordered_map<std::string, double> partials_rmse_orientation;
        std::unordered_map<std::string, double> partials_fps;

        for (auto video_name : test_set)
        {
            std::size_t counter_part = 0;
            std::size_t counter_good_part = 0;
            std::vector<double> errors_part;
            double rmse_position_part = 0;
            double rmse_orientation_part = 0;
            double fps_part = 0;

            SingleVideoResult& video_results = results_.at(video_name);

            for (auto keyframe : keyframes.at(video_name))
            {
                std::size_t excluded_keyframes_range_left = 0;
                std::size_t excluded_keyframes_range_right = 0;

                if (excluded_keyframes.find(video_name) != excluded_keyframes.end())
                {
                    excluded_keyframes_range_left = excluded_keyframes.at(video_name).first;
                    excluded_keyframes_range_right = excluded_keyframes.at(video_name).second;
                }

                if (excluded_keyframes_range_left <= keyframe && keyframe <= excluded_keyframes_range_right)
                    continue;

                Transform<double, 3, Affine> gt_pose = video_results.getGroundTruthPose(keyframe);
                Transform<double, 3, Affine> est_pose = video_results.getObjectPose(keyframe);

                VectorXd error = metric_item.second->evaluate(est_pose, gt_pose, model);

                if (metric_item.first == "ADD-S")
                {
                    // Storage for evaluation of AUC
                    errors_.push_back(error(0));
                    errors_part.push_back(error(0));
                    //

                    // Evaluation of the percentage of metric, i.e. ADD, ADD-S, smaller than max error
                    if (error(0) < max_error)
                    {
                        counter_good++;
                        counter_good_part++;
                    }
                    //
                }
                else if((metric_item.first == "RMS") || (metric_item.first == "RMSSymmetry"))
                {
                    // RMSE
                    rmse_position_ += error(0);
                    rmse_orientation_ += error(1);

                    rmse_position_part += error(0);
                    rmse_orientation_part += error(1);
                    //
                }
                else if (metric_item.first == "FPS")
                {
                    fps_ += video_results.getFPS(keyframe);
                    fps_part += video_results.getFPS(keyframe);
                }

                counter_++;
                counter_part++;
            }

            if (metric_item.first == "ADD-S")
            {
                partials_ratio[video_name] = double(counter_good_part) / double(counter_part);
                partials_auc[video_name] = evaluateAUC(errors_part, 0.1);
            }
            else if((metric_item.first == "RMS") || (metric_item.first == "RMSSymmetry"))
            {
                partials_rmse_position[video_name] = std::sqrt(rmse_position_part / double(counter_part));
                partials_rmse_orientation[video_name] = std::sqrt(rmse_orientation_part / double(counter_part));
            }
            else if(metric_item.first == "FPS")
            {
                partials_fps[video_name] = fps_part / double(counter_part);
            }
        }

        if (metric_item.first == "ADD-S")
        {
            // Print percentage < max_error
            std::cout << "% < " << max_error << "(" << metric_item.first << ") : " << double(counter_good) / double(counter_) * 100.0 << " ";
            map_add_s_less_[object_name] = double(counter_good) / double(counter_) * 100.0;
            std::cout << "{";
            for (auto partial : partials_ratio)
                std::cout << partial.first << ": " << partial.second * 100.0 << " ";
            std::cout << "}" << std::endl;

            // Print AUC (0.1 m)
            double auc = evaluateAUC(errors_, 0.1) * 100.0;
            std::cout << "AUC (0.1 m) (" << metric_item.first << ") : " << auc << " ";
            map_add_s_auc_[object_name] = auc;
            std::cout << "{";
            for (auto partial : partials_auc)
                std::cout << partial.first << ": " << partial.second * 100.0 << " ";
            std::cout << "}" << std::endl;
        }
        else if((metric_item.first == "RMS") || (metric_item.first == "RMSSymmetry"))
        {
            std::cout << "RMSE (position): " << std::setprecision(3) << std::sqrt(rmse_position_ / double(counter_)) * 100.0 << " ";
            map_rmse_position_[object_name] = std::sqrt(rmse_position_ / double(counter_)) * 100.0;
            std::cout << "{";
            for (auto partial : partials_rmse_position)
                std::cout << partial.first << ": " << std::setprecision(3) << partial.second * 100.0 << " ";
            std::cout << "}" << std::endl;

            std::cout << "RMSE (orientation): " << std::setprecision(3) << std::sqrt(rmse_orientation_ / double(counter_)) * 180.0 / M_PI << " ";
            map_rmse_orientation_[object_name] = std::sqrt(rmse_orientation_ / double(counter_)) * 180.0 / M_PI;
            std::cout << "{";
            for (auto partial : partials_rmse_orientation)
                std::cout << partial.first << ": " << std::setprecision(3) << partial.second * 180.0 / M_PI << " ";
            std::cout << "}" << std::endl;
        }
        else if(metric_item.first == "FPS")
        {
            std::cout << "FPS: " << std::setprecision(3) << fps_ / double(counter_) << " ";
            std::cout << "{";
            for (auto partial : partials_fps)
                std::cout << partial.first << ": " << std::setprecision(3) << partial.second << " ";
            std::cout << "}" << std::endl;
        }

        std::cout << std::endl;
    }
}


ObjectResults::ObjectResults
(
    const std::string& object_name,
    const bool& object_has_symmetry,
    const Eigen::Vector3d& object_symmetry_axis,
    const std::string& model_path,
    const std::string& result_path,
    const std::vector<std::string>& test_set,
    const std::size_t skip_frames,
    const std::vector<std::string>& metrics,
    const double& max_error
)
{
    MatrixXd model;
    bool valid_model = false;
    std::tie(valid_model, model) = loadPointCloudFromFile(model_path + object_name);
    if (!valid_model)
        throw(std::runtime_error("[main] Cannot load model of object " + object_name));

    for (auto name : metrics)
    {
        std::string metric_name = name;
        if ((metric_name == "RMSSymmetry") && (!object_has_symmetry))
            metric_name = "RMS";

        if (metric_name == "ADD-S")
            metric_.emplace(metric_name, std::unique_ptr<MetricAddS>(new MetricAddS()));
        else if (metric_name == "RMS")
            metric_.emplace(metric_name, std::unique_ptr<MetricRMS>(new MetricRMS()));
        else if (metric_name == "RMSSymmetry")
            metric_.emplace(metric_name, std::unique_ptr<MetricRMSSymmetry>(new MetricRMSSymmetry(object_symmetry_axis)));
        else if (metric_name == "FPS")
            metric_.emplace(metric_name, std::unique_ptr<MetricEmpty>(new MetricEmpty()));
        else
            throw(std::runtime_error(log_ID_ + "::ctor. Error: cannot load requested metric " + name + "."));

    }

    for (auto video_name : test_set)
        results_.emplace(video_name, SingleVideoResult(object_name, video_name, result_path));

    std::cout << log_ID_ << "::ctor. Object name: " << object_name << std::endl;
    for (auto& metric_item : metric_)
    {
        counter_ = 0;
        std::size_t counter_good = 0;
        rmse_position_ = 0;
        rmse_orientation_ = 0;
        fps_ = 0;

        std::unordered_map<std::string, double> partials_ratio;
        std::unordered_map<std::string, double> partials_auc;
        std::unordered_map<std::string, double> partials_rmse_position;
        std::unordered_map<std::string, double> partials_rmse_orientation;
        std::unordered_map<std::string, double> partials_fps;

        for (auto video_name : test_set)
        {
            std::size_t counter_part = 0;
            std::size_t counter_good_part = 0;
            std::vector<double> errors_part;
            double rmse_position_part = 0;
            double rmse_orientation_part = 0;
            double fps_part = 0;

            SingleVideoResult& video_results = results_.at(video_name);

            for (std::size_t frame = 1 + skip_frames; frame <= video_results.getNumberFrames(); frame++)
            {
                Transform<double, 3, Affine> gt_pose = video_results.getGroundTruthPose(frame);
                Transform<double, 3, Affine> est_pose = video_results.getObjectPose(frame);

                VectorXd error;
                error = metric_item.second->evaluate(est_pose, gt_pose, model);

                if (metric_item.first == "ADD-S")
                {
                    // Storage for evaluation of AUC
                    errors_.push_back(error(0));
                    errors_part.push_back(error(0));
                    //

                    // Evaluation of the percentage of metric, i.e. ADD, ADD-S, smaller than max error
                    if (error(0) < max_error)
                    {
                        counter_good++;
                        counter_good_part++;
                    }
                    //
                }
                else if((metric_item.first == "RMS") || (metric_item.first == "RMSSymmetry"))
                {
                    // RMSE
                    rmse_position_ += error(0);
                    rmse_orientation_ += error(1);

                    rmse_position_part += error(0);
                    rmse_orientation_part += error(1);
                    //
                }
                else if(metric_item.first == "FPS")
                {
                    fps_ += video_results.getFPS(frame);
                    fps_part += video_results.getFPS(frame);
                }

                counter_++;
                counter_part++;
            }

            if ((metric_item.first == "ADD") || (metric_item.first == "ADD-S"))
            {
                partials_ratio[video_name] = double(counter_good_part) / double(counter_part);
                partials_auc[video_name] = evaluateAUC(errors_part, 0.1);
            }
            else if((metric_item.first == "RMS") || (metric_item.first == "RMSSymmetry"))
            {
                partials_rmse_position[video_name] = std::sqrt(rmse_position_part / double(counter_part));
                partials_rmse_orientation[video_name] = std::sqrt(rmse_orientation_part / double(counter_part));
            }
            else if(metric_item.first == "FPS")
            {
                partials_fps[video_name] = fps_part / double(counter_part);
            }
        }

        if (metric_item.first == "ADD-S")
        {
            // Print percentage < max_error
            std::cout << "% < " << max_error << "(" << metric_item.first << ") : " << double(counter_good) / double(counter_) * 100.0 << " ";
            map_add_s_less_[object_name] = double(counter_good) / double(counter_) * 100.0;
            std::cout << "{";
            for (auto partial : partials_ratio)
                std::cout << partial.first << ": " << partial.second * 100.0 << " ";
            std::cout << "}" << std::endl;

            // Print AUC (0.1 m)
            double auc = evaluateAUC(errors_, 0.1) * 100.0;
            std::cout << "AUC (0.1 m) (" << metric_item.first << ") : " << auc << " ";
            map_add_s_auc_[object_name] = auc;
            std::cout << "{";
            for (auto partial : partials_auc)
                std::cout << partial.first << ": " << partial.second * 100.0 << " ";
            std::cout << "}" << std::endl;
        }
        else if((metric_item.first == "RMS") || (metric_item.first == "RMSSymmetry"))
        {
            std::cout << "RMSE (position): " << std::setprecision(3) << std::sqrt(rmse_position_ / double(counter_)) * 100.0 << " ";
            map_rmse_position_[object_name] = std::sqrt(rmse_position_ / double(counter_)) * 100.0;
            std::cout << "{";
            for (auto partial : partials_rmse_position)
                std::cout << partial.first << ": " << std::setprecision(3) << partial.second * 100.0 << " ";
            std::cout << "}" << std::endl;

            std::cout << "RMSE (orientation): " << std::setprecision(3) << std::sqrt(rmse_orientation_ / double(counter_)) * 180.0 / M_PI << " ";
            map_rmse_orientation_[object_name] = std::sqrt(rmse_orientation_ / double(counter_)) * 180.0 / M_PI;
            std::cout << "{";
            for (auto partial : partials_rmse_orientation)
                std::cout << partial.first << ": " << std::setprecision(3) << partial.second * 180.0 / M_PI << " ";
            std::cout << "}" << std::endl;
        }
        else if(metric_item.first == "FPS")
        {
            std::cout << "FPS: " << std::setprecision(3) << fps_ / double(counter_) << " ";
            std::cout << "{";
            for (auto partial : partials_fps)
                std::cout << partial.first << ": " << std::setprecision(3) << partial.second << " ";
            std::cout << "}" << std::endl;
        }

        std::cout << std::endl;
    }
}


ObjectResults::ObjectResults(const std::vector<std::unique_ptr<const ObjectResults>>& results_set, const std::vector<std::string>& metrics, const double& max_error)
{
    std::cout << log_ID_ << "::ctor. MEAN" << std::endl;

    for (const std::string& metric_name : metrics)
    {
        if (metric_name == "ADD-S")
        {
            std::size_t counter = 0;
            std::size_t counter_good = 0;
            std::vector<double> total_errors;

            for (auto& result : results_set)
            {
                std::vector<double> errors;
                std::tie(std::ignore, errors) = result->getErrors();
                counter += errors.size();

                for (std::size_t i = 0; i < errors.size(); i++)
                {
                    total_errors.push_back(errors[i]);
                    if (errors[i] < max_error)
                        counter_good++;
                }
            }

            std::cout << "% < " << max_error << "(" << metric_name << ") : " << double(counter_good) / double(counter) * 100.0 << " " << std::endl;
            double auc = evaluateAUC(total_errors, 0.1) * 100.0;
            std::cout << "AUC (0.1 m) (" << metric_name << ") : " << auc << std::endl;
            map_add_s_auc_["all"] = auc;
            map_add_s_less_["all"] = double(counter_good) / double(counter) * 100.0;
        }
        else if((metric_name == "RMS") || (metric_name == "RMSSymmetry"))
        {
            std::size_t counter = 0;
            double total_rmse_position = 0;
            double total_rmse_orientation = 0;
            for (auto& result : results_set)
            {
                double rmse_position;
                double rmse_orientation;
                std::size_t number_samples;
                std::tie(number_samples, rmse_position, rmse_orientation) = result->getRMSE();

                counter += number_samples;
                total_rmse_position += rmse_position;
                total_rmse_orientation += rmse_orientation;
            }

            std::cout << "RMSE (position): " << std::setprecision(3) << std::sqrt(total_rmse_position / double(counter)) * 100.0 << std::endl;
            std::cout << "RMSE (orientation): " << std::setprecision(3) << std::sqrt(total_rmse_orientation / double(counter)) * 180.0 / M_PI << std::endl;
            map_rmse_position_["all"] = std::sqrt(total_rmse_position / double(counter)) * 100.0;
            map_rmse_orientation_["all"] = std::sqrt(total_rmse_orientation / double(counter)) * 180.0 / M_PI;
        }
        else if(metric_name == "FPS")
        {
            std::size_t counter = 0;
            double total_fps = 0;
            for (auto& result : results_set)
            {
                double fps;
                std::size_t number_samples;
                std::tie(number_samples, fps) = result->getFPS();

                counter += number_samples;
                total_fps += fps;
            }

            map_fps_["all"] = total_fps / double(counter);

            std::cout << "FPS: " << std::setprecision(3) << total_fps / double(counter) << std::endl;
        }
    }
}


std::tuple<std::size_t, std::vector<double>> ObjectResults::getErrors() const
{
    return std::make_tuple(counter_, errors_);
}


std::tuple<std::size_t, double, double> ObjectResults::getRMSE() const
{
    return std::make_tuple(counter_, rmse_position_, rmse_orientation_);
}


std::tuple<std::size_t, double> ObjectResults::getFPS() const
{
    return std::make_tuple(counter_, fps_);
}


double ObjectResults::evaluateAUC(const std::vector<double>& errors, const double& max_threshold)
{
    /* Adapted from https://github.com/yuxng/YCB_Video_toolbox/blob/d08b645d406b93a988087fea42a5f6ac7330933c/plot_accuracy_keyframe.m#L143. */
    std::vector<double> metrics = errors;
    for (std::size_t i = 0; i < metrics.size(); i++)
        if (metrics[i] > max_threshold)
            metrics[i] = std::numeric_limits<double>::infinity();

    std::sort(metrics.begin(), metrics.end());

    VectorXd accuracy_classes(metrics.size());
    for (std::size_t i = 0; i < metrics.size(); i++)
        accuracy_classes(i) = double(i + 1) / double(metrics.size());

    std::size_t counter = 0;
    for (std::size_t i = 0; i < metrics.size(); i++)
        if (metrics[i] != std::numeric_limits<double>::infinity())
            counter++;

    VectorXd accuracy_classes_modified(counter + 2);
    accuracy_classes_modified(0) = 0.0;
    accuracy_classes_modified.segment(1, counter) = accuracy_classes.segment(0, counter);
    accuracy_classes_modified(accuracy_classes_modified.size() - 1) = accuracy_classes(counter - 1);

    VectorXd metrics_modified(counter + 2);
    Map<const VectorXd> metrics_eigen(metrics.data(), metrics.size());
    metrics_modified(0) = 0.0;
    metrics_modified.segment(1, counter) = metrics_eigen.segment(0, counter);
    metrics_modified(metrics_modified.size() - 1) = 0.1;

    for (std::size_t i = 1; i < accuracy_classes_modified.size(); i++)
        accuracy_classes_modified(i) = std::max(accuracy_classes_modified(i), accuracy_classes_modified(i - 1));

    std::vector<std::size_t> indexes;
    for (std::size_t i = 1; i < metrics_modified.size(); i++)
        if (metrics_modified(i) != metrics_modified(i - 1))
            indexes.push_back(i);
    double sum = 0;
    for (std::size_t i = 0; i < indexes.size(); i++)
        sum += (metrics_modified(indexes[i]) - metrics_modified(indexes[i] - 1)) * accuracy_classes_modified(indexes[i]);
    sum *= 10;

    return sum;
}


std::pair<bool, MatrixXd> ObjectResults::loadPointCloudFromFile(const std::string& filename)
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
