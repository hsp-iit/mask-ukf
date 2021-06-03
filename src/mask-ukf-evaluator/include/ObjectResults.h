/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef OBJECTRESULTS_H
#define OBJECTRESULTS_H

#include <Eigen/Dense>

#include <Metric.h>
#include <Results.h>
#include <SingleVideoResult.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>


class ObjectResults : public Results
{
public:
    ObjectResults
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
        );

    ObjectResults
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
     );

    ObjectResults(const std::vector<std::unique_ptr<const ObjectResults>>& results_set, const std::vector<std::string>& metrics, const double& max_error);

    std::tuple<std::size_t, std::vector<double>> getErrors() const;

    std::tuple<std::size_t, double, double> getRMSE() const;

    std::tuple<std::size_t, double, double> getRMSEV() const;

    std::tuple<std::size_t, double> getFPS() const;

private:
    double evaluateAUC(const std::vector<double>& errors, const double& max_threshold);

    std::pair<bool, Eigen::MatrixXd> loadPointCloudFromFile(const std::string& filename);

    std::unordered_map<std::string, SingleVideoResult> results_;

    std::unordered_map<std::string, std::unique_ptr<Metric>> metric_;

    std::vector<double> errors_;

    double rmse_position_;

    double rmse_orientation_;

    double fps_;

    std::size_t counter_;

    const std::string log_ID_ = "[ObjectResults]";
};

#endif /* OBJECTRESULTS_H */
