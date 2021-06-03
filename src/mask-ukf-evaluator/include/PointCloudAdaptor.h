/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef POINTCLOUDADAPTOR_H
#define POINTCLOUDADAPTOR_H

struct PointCloudAdaptor
{
    const Eigen::Ref<const Eigen::MatrixXd> data;

    PointCloudAdaptor(const Eigen::Ref<const Eigen::MatrixXd>& data_) : data(data_) { }

    /// CRTP helper method
    inline Eigen::Ref<const Eigen::MatrixXd> derived() const { return data; }

    // Must return the number of data points
    inline std::size_t kdtree_get_point_count() const { return data.cols(); }

    // Returns the dim'th component of the idx'th point in the class:
    inline double kdtree_get_pt(const size_t idx, const size_t dim) const
    {
        return derived()(dim, idx);
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
    //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /*bb*/) const { return false; }
};

#endif
