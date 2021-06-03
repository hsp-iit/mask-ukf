/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <Results.h>

#include <map>

class ResultsPoseRBPFADDS : public Results
{
public:
    ResultsPoseRBPFADDS()
    {
        map_add_s_auc_ =
        {
            {"002_master_chef_can", 95.1},
            {"003_cracker_box", 93.0},
            {"004_sugar_box", 95.5},
            {"005_tomato_soup_can", 93.8},
            {"006_mustard_bottle", 96.3},
            {"007_tuna_fish_can", 95.3},
            {"008_pudding_box", 92.0},
            {"009_gelatin_box", 97.5},
            {"010_potted_meat_can", 77.9},
            {"011_banana", 86.9},
            {"019_pitcher_base", 94.2},
            {"021_bleach_cleanser", 93.0},
            {"024_bowl", 94.2},
            {"025_mug", 97.1},
            {"035_power_drill", 96.1},
            {"036_wood_block", 89.1},
            {"037_scissors", 85.6},
            {"040_large_marker", 97.1},
            {"051_large_clamp", 94.8},
            {"052_extra_large_clamp", 90.1},
            {"061_foam_brick", 95.7},
            {"all", 93.3}
        };
    }
};
