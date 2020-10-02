/**
 * @file    CalibrationHandler.cpp
 *
 * @author  btran
 *
 */

#include <boost/math/constants/constants.hpp>

#include <sensors_calib/CalibrationHandler.hpp>

namespace perception
{
CalibrationHandlerParam getCalibrationHandlerParam(const std::string& jsonPath)
{
    rapidjson::Document jsonDoc = readFromJsonFile(jsonPath);
    CalibrationHandlerParam param;

    param.pathToInitialGuess = getValueAs<std::string>(jsonDoc, "path_to_initial_guess");
    param.pathToImages = getValueAs<std::string>(jsonDoc, "path_to_images");
    param.pathToPointClouds = getValueAs<std::string>(jsonDoc, "path_to_point_clouds");
    param.pathToCameraInfo = getValueAs<std::string>(jsonDoc, "path_to_camera_info");

    param.numBins = getValueAs<int>(jsonDoc, "num_bins");
    param.deltaTrans = getValueAs<double>(jsonDoc, "delta_trans");
    param.deltaRotRad = getValueAs<double>(jsonDoc, "delta_rot_deg") * boost::math::double_constants::degree;
    param.deltaStepFactor = getValueAs<double>(jsonDoc, "delta_step_factor");
    param.deltaThresh = getValueAs<double>(jsonDoc, "delta_thresh");
    param.gammaTransU = getValueAs<double>(jsonDoc, "gamma_trans_upper");
    param.gammaTransL = getValueAs<double>(jsonDoc, "gamma_trans_lower");
    param.gammaRotU = getValueAs<double>(jsonDoc, "gamma_rot_upper") * boost::math::double_constants::degree;
    param.gammaRotL = getValueAs<double>(jsonDoc, "gamma_rot_lower") * boost::math::double_constants::degree;
    param.gammaStepFactor = getValueAs<double>(jsonDoc, "gamma_step_factor");
    param.maxIter = getValueAs<std::size_t>(jsonDoc, "max_iter");
    param.xMin = getValueAs<double>(jsonDoc, "x_min");
    param.xMax = getValueAs<double>(jsonDoc, "x_max");
    param.yMin = getValueAs<double>(jsonDoc, "y_min");
    param.yMax = getValueAs<double>(jsonDoc, "y_max");
    param.zMin = getValueAs<double>(jsonDoc, "z_min");
    param.zMax = getValueAs<double>(jsonDoc, "z_max");
    param.filterInputImage = getValueAs<bool>(jsonDoc, "filter_input_image");
    param.filterDiameter = getValueAs<int>(jsonDoc, "filter_diameter");
    param.sigmaColor = getValueAs<double>(jsonDoc, "sigma_color");
    param.sigmaSpace = getValueAs<double>(jsonDoc, "sigma_space");
    param.normalizeMI = getValueAs<bool>(jsonDoc, "normalize_mi");
    param.probabilityEstimatorType = getValueAs<int>(jsonDoc, "probability_estimator_type");
    param.useBayes = getValueAs<bool>(jsonDoc, "use_bayes");

    return param;
}

template <> void validate<CalibrationHandlerParam>(const CalibrationHandlerParam& param)
{
    if (param.pathToImages.empty()) {
        throw std::runtime_error("empty path to images");
    }

    if (param.pathToPointClouds.empty()) {
        throw std::runtime_error("empty path to point clouds");
    }

    if (param.pathToCameraInfo.empty()) {
        throw std::runtime_error("empty path to camera info");
    }
}
}  // namespace perception
