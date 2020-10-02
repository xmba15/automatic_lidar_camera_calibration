/**
 * @file    Types.cpp
 *
 * @author  btran
 *
 */

#include <sensors_calib/Types.hpp>
#include <sensors_calib/utils/ParamConfig.hpp>

namespace perception
{
CameraInfo::CameraInfo(const std::string cameraInfoPath)
    : m_K(Eigen::Matrix3d::Zero())
{
    rapidjson::Document jsonDoc = readFromJsonFile(cameraInfoPath);
    const rapidjson::Value& K = jsonDoc["K"];
    if (!K.IsArray()) {
        throw std::runtime_error("failed to get K");
    }

    if (static_cast<int>(K.Size()) != 9) {
        throw std::runtime_error("invalid K");
    }

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            m_K(i, j) = K[static_cast<rapidjson::SizeType>(i * 3 + j)].GetDouble();
        }
    }
}

CameraInfo::~CameraInfo()
{
}

const Eigen::Matrix3d& CameraInfo::K() const
{
    return m_K;
}
}  // namespace perception
