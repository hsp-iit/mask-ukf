/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ObjectResults.h>

#include <Eigen/Dense>

#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

using namespace Eigen;


std::pair<bool, std::unordered_map<std::string, std::vector<std::size_t>>> loadKeyframes(const std::string& filename);


int main(int argc, char** argv)
{
    if ((argc != 5) && (argc != 6))
    {
        std::cout << "Synopsis: validation <metric> <results_path> <models_path> <use_key_frames> <keyframes_path>" << std::endl;
        exit(EXIT_FAILURE);
    }

    const std::string metric = std::string(argv[1]);
    const std::string result_path = std::string(argv[2]);
    const std::string models_path = std::string(argv[3]);
    const bool use_keyframes = (std::string(argv[4]) == "true");
    std::string keyframes_path;
    if (use_keyframes)
        keyframes_path = std::string(argv[5]);
    const double max_error = 0.02;

    /* Metrics. */
    const std::vector<std::string> metrics {metric};

    /* Keyframes. */
    bool valid_keyframes = false;
    std::unordered_map<std::string, std::vector<std::size_t>> keyframes;
    if (use_keyframes)
        std::tie(valid_keyframes, keyframes) = loadKeyframes(keyframes_path);
    if (use_keyframes && (!valid_keyframes))
    {
        std::cout << "Cannot load keyframes file." << std::endl;
        exit(EXIT_FAILURE);
    }
    std::unordered_map<std::string, std::pair<std::size_t, std::size_t>> skipped_frames_placeholder;

    /* Dictionary. */
    std::vector<std::string> keys = {"002_master_chef_can", "003_cracker_box",
                                     "004_sugar_box",       "005_tomato_soup_can",
                                     "006_mustard_bottle",  "007_tuna_fish_can",
                                     "008_pudding_box",     "009_gelatin_box",
                                     "010_potted_meat_can", "011_banana",
                                     "019_pitcher_base",    "021_bleach_cleanser",
                                     "024_bowl",            "025_mug",
                                     "035_power_drill",     "036_wood_block",
                                     "037_scissors",        "040_large_marker",
                                     "051_large_clamp",     "052_extra_large_clamp",
                                     "061_foam_brick"};

    std::unordered_map<std::string, std::vector<std::string>> objects_videos;
    objects_videos["002_master_chef_can"]   = std::vector<std::string>({"0048", "0051", "0055", "0056"});
    objects_videos["003_cracker_box"]       = std::vector<std::string>({"0050", "0054", "0059"});
    objects_videos["004_sugar_box"]         = std::vector<std::string>({"0049", "0051", "0054", "0055", "0058"});
    objects_videos["005_tomato_soup_can"]   = std::vector<std::string>({"0050", "0051", "0053", "0055", "0057", "0059"});
    objects_videos["006_mustard_bottle"]    = std::vector<std::string>({"0050", "0052"});
    objects_videos["007_tuna_fish_can"]     = std::vector<std::string>({"0048", "0049", "0052", "0059"});
    objects_videos["008_pudding_box"]       = std::vector<std::string>({"0058"});
    objects_videos["009_gelatin_box"]       = std::vector<std::string>({"0058"});
    objects_videos["010_potted_meat_can"]   = std::vector<std::string>({"0049", "0053", "0059"});
    objects_videos["011_banana"]            = std::vector<std::string>({"0050", "0056"});
    objects_videos["019_pitcher_base"]      = std::vector<std::string>({"0052", "0056", "0058"});
    objects_videos["021_bleach_cleanser"]   = std::vector<std::string>({"0051", "0054", "0055", "0057"});
    objects_videos["024_bowl"]              = std::vector<std::string>({"0049", "0053"});
    objects_videos["025_mug"]               = std::vector<std::string>({"0048", "0055"});
    objects_videos["035_power_drill"]       = std::vector<std::string>({"0050", "0054", "0056", "0059"});
    objects_videos["036_wood_block"]        = std::vector<std::string>({"0055"});
    objects_videos["037_scissors"]          = std::vector<std::string>({"0051"});
    objects_videos["040_large_marker"]      = std::vector<std::string>({"0057", "0059"});
    objects_videos["051_large_clamp"]       = std::vector<std::string>({"0048", "0054"});
    objects_videos["052_extra_large_clamp"] = std::vector<std::string>({"0048", "0057"});
    objects_videos["061_foam_brick"]        = std::vector<std::string>({"0057"});

    std::unordered_map<std::string, bool> objects_symmetry;
    objects_symmetry["002_master_chef_can"]   = true;
    objects_symmetry["003_cracker_box"]       = false;
    objects_symmetry["004_sugar_box"]         = false;
    objects_symmetry["005_tomato_soup_can"]   = true;
    objects_symmetry["006_mustard_bottle"]    = false;
    objects_symmetry["007_tuna_fish_can"]     = true;
    objects_symmetry["008_pudding_box"]       = false;
    objects_symmetry["009_gelatin_box"]       = false;
    objects_symmetry["010_potted_meat_can"]   = false;
    objects_symmetry["011_banana"]            = false;
    objects_symmetry["019_pitcher_base"]      = false;
    objects_symmetry["021_bleach_cleanser"]   = false;
    objects_symmetry["024_bowl"]              = true;
    objects_symmetry["025_mug"]               = false;
    objects_symmetry["035_power_drill"]       = false;
    objects_symmetry["036_wood_block"]        = true;
    objects_symmetry["037_scissors"]          = false;
    objects_symmetry["040_large_marker"]      = true;
    objects_symmetry["051_large_clamp"]       = true;
    objects_symmetry["052_extra_large_clamp"] = true;
    objects_symmetry["061_foam_brick"]        = false;

    std::unordered_map<std::string, Eigen::Vector3d> objects_symmetry_axes;
    objects_symmetry_axes["002_master_chef_can"]   = Vector3d::UnitZ();
    objects_symmetry_axes["003_cracker_box"]       = Vector3d::Zero();
    objects_symmetry_axes["004_sugar_box"]         = Vector3d::Zero();
    objects_symmetry_axes["005_tomato_soup_can"]   = Vector3d::UnitZ();
    objects_symmetry_axes["006_mustard_bottle"]    = Vector3d::Zero();
    objects_symmetry_axes["007_tuna_fish_can"]     = Vector3d::UnitZ();
    objects_symmetry_axes["008_pudding_box"]       = Vector3d::Zero();
    objects_symmetry_axes["009_gelatin_box"]       = Vector3d::Zero();
    objects_symmetry_axes["010_potted_meat_can"]   = Vector3d::Zero();
    objects_symmetry_axes["011_banana"]            = Vector3d::Zero();
    objects_symmetry_axes["019_pitcher_base"]      = Vector3d::Zero();
    objects_symmetry_axes["021_bleach_cleanser"]   = Vector3d::Zero();
    objects_symmetry_axes["024_bowl"]              = Vector3d::UnitZ();
    objects_symmetry_axes["025_mug"]               = Vector3d::Zero();
    objects_symmetry_axes["035_power_drill"]       = Vector3d::Zero();
    objects_symmetry_axes["036_wood_block"]        = Vector3d::UnitZ();
    objects_symmetry_axes["037_scissors"]          = Vector3d::Zero();
    objects_symmetry_axes["040_large_marker"]      = Vector3d::UnitY();
    objects_symmetry_axes["051_large_clamp"]       = Vector3d::UnitY();
    objects_symmetry_axes["052_extra_large_clamp"] = Vector3d::UnitX();
    objects_symmetry_axes["061_foam_brick"]        = Vector3d::Zero();

    std::vector<std::unique_ptr<const ObjectResults>> objects_results(keys.size());

    if (use_keyframes)
        for (std::size_t i = 0; i < keys.size(); i++)
            objects_results[i] = std::unique_ptr<ObjectResults>(new ObjectResults(keys[i], objects_symmetry[keys[i]], objects_symmetry_axes[keys[i]], models_path, result_path, objects_videos[keys[i]], keyframes, skipped_frames_placeholder, metrics, max_error));
    else
        for (std::size_t i = 0; i < keys.size(); i++)
            objects_results[i] = std::unique_ptr<ObjectResults>(new ObjectResults(keys[i], objects_symmetry[keys[i]], objects_symmetry_axes[keys[i]], models_path, result_path, objects_videos[keys[i]], 1, metrics, max_error));

    ObjectResults (objects_results, metrics, max_error);

    return EXIT_SUCCESS;
}


std::pair<bool, std::unordered_map<std::string, std::vector<std::size_t>>> loadKeyframes(const std::string& filename)
{
    std::unordered_map<std::string, std::vector<std::size_t>> keyframes;

    std::ifstream in(filename);

    if (!in.is_open())
    {
        std::cout << "loadKeyframes. Failed to open " << filename << '\n';

        return std::make_pair(false, keyframes);
    }

    std::string line;
    while (std::getline(in, line))
    {
        std::istringstream line_stream(line);
        std::vector<std::string> split;
        std::string item;
        while (getline(line_stream, item, '/'))
            split.push_back(item);

        std::stringstream sstream(split.at(1));
        size_t frame_number;
        sstream >> frame_number;

        keyframes[split.at(0)].push_back(frame_number);
    }

    return std::make_pair(true, keyframes);
}
