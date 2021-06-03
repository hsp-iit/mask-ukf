/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef RESULTS_H
#define RESULTS_H

#include <map>

class Results
{
public:
    std::map<std::string, double> map_rmse_position_;

    std::map<std::string, double> map_rmse_orientation_;

    std::map<std::string, double> map_add_s_auc_;

    std::map<std::string, double> map_add_s_less_;

    std::map<std::string, double> map_fps_;
};

#endif
