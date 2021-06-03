/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <Results.h>

#include <map>

class ResultsDenseFusionADDS : public Results
{
public:
    ResultsDenseFusionADDS()
    {
        map_add_s_auc_ =
        {
            {"002_master_chef_can", 96.4},
            {"003_cracker_box", 95.5},
            {"004_sugar_box", 97.5},
            {"005_tomato_soup_can", 94.6},
            {"006_mustard_bottle", 97.2},
            {"007_tuna_fish_can", 96.6},
            {"008_pudding_box", 96.5},
            {"009_gelatin_box", 98.1},
            {"010_potted_meat_can", 91.3},
            {"011_banana", 96.6},
            {"019_pitcher_base", 97.1},
            {"021_bleach_cleanser", 95.8},
            {"024_bowl", 88.2},
            {"025_mug", 97.1},
            {"035_power_drill", 96.0},
            {"036_wood_block", 89.7},
            {"037_scissors", 95.2},
            {"040_large_marker", 97.5},
            {"051_large_clamp", 72.9},
            {"052_extra_large_clamp", 69.8},
            {"061_foam_brick", 92.5},
            {"all", 93.1}
        };

        map_add_s_less_ =
        {
            {"002_master_chef_can", 100.0},
            {"003_cracker_box", 99.5},
            {"004_sugar_box", 100.0},
            {"005_tomato_soup_can", 96.9},
            {"006_mustard_bottle", 100.0},
            {"007_tuna_fish_can", 100.0},
            {"008_pudding_box", 100.0},
            {"009_gelatin_box", 100.0},
            {"010_potted_meat_can", 93.1},
            {"011_banana", 100.0},
            {"019_pitcher_base", 100.0},
            {"021_bleach_cleanser", 100.0},
            {"024_bowl", 98.8},
            {"025_mug", 100.0},
            {"035_power_drill", 98.7},
            {"036_wood_block", 94.6},
            {"037_scissors", 100.0},
            {"040_large_marker", 100.0},
            {"051_large_clamp", 79.2},
            {"052_extra_large_clamp", 76.3},
            {"061_foam_brick", 100.0},
            {"all", 96.8}
        };
    }
};
