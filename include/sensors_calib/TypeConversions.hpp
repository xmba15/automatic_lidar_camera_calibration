/**
 * @file    TypeConversions.hpp
 *
 * @author  btran
 *
 */

#pragma once

#include "Types.hpp"

namespace perception
{
inline Eigen::Affine3d toAffine(const TransformInfo& transformInfo)
{
    Eigen::Affine3d affine = Eigen::Affine3d::Identity();
    affine.translation() << transformInfo(0), transformInfo(1), transformInfo(2);
    Eigen::Quaterniond quat(Eigen::AngleAxisd(transformInfo[3], Eigen::Vector3d::UnitX()) *
                            Eigen::AngleAxisd(transformInfo[4], Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(transformInfo[5], Eigen::Vector3d::UnitZ()));
    affine.linear() = quat.matrix();
    return affine;
}
}  // namespace perception
