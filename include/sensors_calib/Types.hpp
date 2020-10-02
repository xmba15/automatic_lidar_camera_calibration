/**
 * @file    Types.hpp
 *
 * @author  btran
 *
 */

#pragma once

#include <memory>
#include <string>

#include <Eigen/Core>
#include <Eigen/Dense>

namespace perception
{
class CameraInfo
{
 public:
    using Ptr = std::shared_ptr<CameraInfo>;

    explicit CameraInfo(const std::string cameraInfoPath);
    ~CameraInfo();

    const Eigen::Matrix3d& K() const;

 private:
    Eigen::Matrix3d m_K;
};

using TransformInfo = Eigen::Matrix<double, 6, 1>;  // x, y, z, r, p, y
}  // namespace perception
