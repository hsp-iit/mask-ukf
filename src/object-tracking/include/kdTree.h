/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef KDTREE_H
#define KDTREE_H

#include <PointCloudAdaptor.h>

#include <nanoflann.hpp>

using kdTree = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, PointCloudAdaptor >,
                                                   PointCloudAdaptor,
                                                   3 /* dimension, since using point clouds */>;
#endif
