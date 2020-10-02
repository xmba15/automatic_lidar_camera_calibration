/**
 * @file    Utility.cpp
 *
 * @author  btran
 *
 */

#include <boost/math/constants/constants.hpp>

#include <sensors_calib/utils/ParamConfig.hpp>
#include <sensors_calib/utils/Utility.hpp>

namespace perception
{
TransformInfo getTransformInfo(const std::string transformationInfoPath)
{
    rapidjson::Document jsonDoc = readFromJsonFile(transformationInfoPath);

    TransformInfo info;
    info(0) = getValueAs<double>(jsonDoc, "x");
    info(1) = getValueAs<double>(jsonDoc, "y");
    info(2) = getValueAs<double>(jsonDoc, "z");

    // change to radian
    info(3) = getValueAs<double>(jsonDoc, "r_deg") * boost::math::double_constants::degree;
    info(4) = getValueAs<double>(jsonDoc, "p_deg") * boost::math::double_constants::degree;
    info(5) = getValueAs<double>(jsonDoc, "y_deg") * boost::math::double_constants::degree;

    return info;
}
}  // namespace perception
