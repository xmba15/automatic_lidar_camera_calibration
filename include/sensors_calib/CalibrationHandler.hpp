/**
 * @file    CalibrationHandler.hpp
 *
 * @author  btran
 *
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include <pcl/io/pcd_io.h>

#include <boost/math/constants/constants.hpp>

#include "HistogramHandler.hpp"
#include "ProbabilityHandler.hpp"
#include "TypeConversions.hpp"
#include "Types.hpp"
#include "utils/utils.hpp"

namespace perception
{
struct CalibrationHandlerParam {
    std::string pathToInitialGuess = "";
    std::string pathToImages = "";
    std::string pathToPointClouds = "";
    std::string pathToCameraInfo = "";
    int numBins = 256;

    // optimization param
    double deltaTrans = 0.01;
    double deltaRotRad = 0.1 * boost::math::double_constants::degree;
    double deltaStepFactor = 1.1;
    double deltaThresh = 0.0001;
    double gammaTransU = 0.1;
    double gammaTransL = 0.01;
    double gammaRotU = 3.14 * boost::math::double_constants::degree;
    double gammaRotL = 0.1 * boost::math::double_constants::degree;
    double gammaStepFactor = 1.2;
    std::size_t maxIter = 300;

    // point cloud xyz filter
    double xMin = 0;
    double xMax = 10;
    double yMin = -10;
    double yMax = 10;
    double zMin = -10;
    double zMax = 10;

    double epsilon = 1e-9;  // epsilon to avoid division by 0

    // filter input images
    bool filterInputImage = true;  // use bilateral filter
    int filterDiameter = 15;
    double sigmaColor = 75;
    double sigmaSpace = 75;

    // mutual information related params
    bool normalizeMI = false;
    int probabilityEstimatorType = 0;
    bool useBayes = false;
};

CalibrationHandlerParam getCalibrationHandlerParam(const std::string& jsonPath);
template <> void validate<CalibrationHandlerParam>(const CalibrationHandlerParam& param);

template <typename POINT_CLOUD_TYPE> class CalibrationHandler
{
 public:
    using PointCloudType = POINT_CLOUD_TYPE;
    using PointCloud = pcl::PointCloud<PointCloudType>;
    using PointCloudPtr = typename PointCloud::Ptr;

    using Ptr = std::shared_ptr<CalibrationHandler>;
    using DeltaTransformInfo = TransformInfo;

 public:
    explicit CalibrationHandler(const CalibrationHandlerParam& param);
    virtual ~CalibrationHandler();

    TransformInfo optimize();

    std::vector<cv::Mat> drawPointCloudOnImagePlane(const TransformInfo& transform) const;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> projectOnPointCloud(const TransformInfo& transform) const;

 private:
    /**
     *  @brief calculate MI cost with the current param
     */
    double calculateMICost(const TransformInfo& transform);
    DeltaTransformInfo step(const double prevCost, const TransformInfo& transform,
                            const CalibrationHandlerParam& param);

 private:
    CalibrationHandlerParam m_param;

    TransformInfo m_initialGuess;
    TransformInfo m_transformation;  // final transformation parameter
    CameraInfo::Ptr m_cameraInfo;

    HistogramHandler::Ptr m_histogramHandler;
    ProbabilityHandler::Ptr m_probabilityHandler;

    std::vector<cv::Mat> m_colorImgs;
    std::vector<cv::Mat> m_grayImgs;
    std::vector<PointCloudPtr> m_poinclouds;
};

}  // namespace perception
#include "impl/CalibrationHandler.ipp"
