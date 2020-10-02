/**
 * @file    CalibrationHandler.ipp
 *
 * @author  btran
 *
 */

namespace perception
{
template <typename POINT_CLOUD_TYPE>
CalibrationHandler<POINT_CLOUD_TYPE>::CalibrationHandler(const CalibrationHandlerParam& param)
    : m_param(param)
    , m_initialGuess(TransformInfo::Identity())
{
    validate<CalibrationHandlerParam>(m_param);

    if (!param.pathToInitialGuess.empty()) {
        m_initialGuess = perception::getTransformInfo(m_param.pathToInitialGuess);
    }
    std::vector<std::string> imagePaths = parseMetaDataFile(m_param.pathToImages);
    std::vector<std::string> pointcloudPaths = parseMetaDataFile(m_param.pathToPointClouds);

    if (imagePaths.size() != pointcloudPaths.size()) {
        throw std::runtime_error("number of images and point clouds must be the same");
    }

    if (imagePaths.empty()) {
        throw std::runtime_error("empty calibration data");
    }

    const int numSamples = imagePaths.size();
    m_colorImgs.reserve(numSamples);
    m_grayImgs.reserve(numSamples);
    m_poinclouds.reserve(numSamples);

    cv::Mat colorImg, grayImg;
    PointCloudPtr inCloud(new PointCloud);

    for (int i = 0; i < numSamples; ++i) {
        colorImg = cv::imread(imagePaths[i]);

        if (colorImg.empty()) {
            throw std::runtime_error("failed to read: " + imagePaths[i]);
        }

        if (pcl::io::loadPCDFile<PointCloudType>(pointcloudPaths[i], *inCloud) == -1) {
            throw std::runtime_error("failed to read: " + pointcloudPaths[i]);
        }

        if (m_param.filterInputImage) {
            cv::Mat filtered;
            cv::bilateralFilter(colorImg, filtered, m_param.filterDiameter, m_param.sigmaColor, m_param.sigmaSpace);
            cv::cvtColor(filtered, grayImg, cv::COLOR_BGR2GRAY);
        } else {
            cv::cvtColor(colorImg, grayImg, cv::COLOR_BGR2GRAY);
        }

        m_colorImgs.emplace_back(colorImg);
        m_grayImgs.emplace_back(grayImg);

        inCloud = perception::PointCloudFilter<PointCloudType>::filterXYZAxis(
            inCloud, m_param.xMin, m_param.xMax, m_param.yMin, m_param.yMax, m_param.zMin, m_param.zMax);

        m_poinclouds.emplace_back(PointCloudPtr(new PointCloud(*inCloud)));
    }

    m_cameraInfo.reset(new CameraInfo(m_param.pathToCameraInfo));
    m_histogramHandler.reset(new perception::HistogramHandler(m_param.numBins));
    m_probabilityHandler.reset(new perception::ProbabilityHandler(m_param.numBins));
}

template <typename POINT_CLOUD_TYPE> CalibrationHandler<POINT_CLOUD_TYPE>::~CalibrationHandler()
{
}

template <typename POINT_CLOUD_TYPE>
double CalibrationHandler<POINT_CLOUD_TYPE>::calculateMICost(const TransformInfo& transform)
{
    m_histogramHandler.reset(new perception::HistogramHandler(m_param.numBins));
    m_probabilityHandler.reset(new perception::ProbabilityHandler(m_param.numBins));

    m_histogramHandler->update<PointCloudType>(m_grayImgs, m_poinclouds, *m_cameraInfo,
                                               perception::toAffine(transform));

    switch (m_param.probabilityEstimatorType) {
        case 0: {
            m_probabilityHandler->estimateMLE(m_histogramHandler, m_param.useBayes);
            break;
        }
        case 1:
        default:
            m_probabilityHandler->estimateJS(m_histogramHandler, m_param.useBayes);
            break;
    }

    return m_probabilityHandler->calculateMICost(m_param.normalizeMI);
}

template <typename POINT_CLOUD_TYPE>
typename CalibrationHandler<POINT_CLOUD_TYPE>::DeltaTransformInfo
CalibrationHandler<POINT_CLOUD_TYPE>::step(const double prevCost, const TransformInfo& transform,
                                           const CalibrationHandlerParam& param)
{
    DeltaTransformInfo deltaTransformInfo = DeltaTransformInfo::Zero();
    TransformInfo deltaValues;
    deltaValues << param.deltaTrans, param.deltaTrans, param.deltaTrans, param.deltaRotRad, param.deltaRotRad,
        param.deltaRotRad;

    for (int i = 0; i < deltaValues.size(); ++i) {
        TransformInfo curTransform = transform;
        curTransform(i) += deltaValues(i);
        double curCost = this->calculateMICost(curTransform);
        deltaTransformInfo(i) = (curCost - prevCost) / deltaValues(i);
    }

    deltaTransformInfo.segment(0, 3).normalize();
    deltaTransformInfo.segment(3, 3).normalize();

    DEBUG_LOG("delta transform: x:%f, y:%f, z:%f, r:%f, p:%f, y:%f\n", deltaTransformInfo(0), deltaTransformInfo(1),
              deltaTransformInfo(2), deltaTransformInfo(3), deltaTransformInfo(4), deltaTransformInfo(5));
    return deltaTransformInfo;
}

template <typename POINT_CLOUD_TYPE> TransformInfo CalibrationHandler<POINT_CLOUD_TYPE>::optimize()
{
    TransformInfo prevTransform = TransformInfo::Zero(), curTransform = m_initialGuess;
    DeltaTransformInfo prevDeltaTransform = DeltaTransformInfo::Zero(), curDeltaTransform = DeltaTransformInfo::Zero();

    double gammaTrans, gammaRot;

    for (std::size_t i = 0; i < m_param.maxIter; ++i) {
        double prevCost = this->calculateMICost(curTransform);
        curDeltaTransform = this->step(prevCost, curTransform, m_param);

        if ((curTransform.segment(0, 3) - prevTransform.segment(0, 3)).norm() > 0) {
            gammaTrans = (curTransform.segment(0, 3) - prevTransform.segment(0, 3))
                             .dot(curTransform.segment(0, 3) - prevTransform.segment(0, 3)) /
                         (((curTransform.segment(0, 3) - prevTransform.segment(0, 3))
                               .dot(curDeltaTransform.segment(0, 3) - prevDeltaTransform.segment(0, 3))) +
                          m_param.epsilon);
        } else {
            gammaTrans = m_param.gammaTransU;
        }

        if ((curTransform.segment(3, 3) - prevTransform.segment(3, 3)).norm() > 0) {
            gammaRot = (curTransform.segment(3, 3) - prevTransform.segment(3, 3))
                           .dot(curTransform.segment(3, 3) - prevTransform.segment(3, 3)) /
                       (((curTransform.segment(3, 3) - prevTransform.segment(3, 3))
                             .dot(curDeltaTransform.segment(3, 3) - prevDeltaTransform.segment(3, 3))) +
                        m_param.epsilon);
        } else {
            gammaRot = m_param.gammaRotU;
        }

        gammaTrans = std::clamp(gammaTrans, m_param.gammaTransL, m_param.gammaTransU);
        gammaRot = std::clamp(gammaRot, m_param.gammaRotL, m_param.gammaRotU);

        prevTransform = curTransform;
        curTransform.segment(0, 3) += gammaTrans * curDeltaTransform.segment(0, 3);
        curTransform.segment(3, 3) += gammaRot * curDeltaTransform.segment(3, 3);

        double curCost = this->calculateMICost(curTransform);
        if (curCost < prevCost) {
            curTransform = prevTransform;
            m_param.deltaTrans /= m_param.deltaStepFactor;
            m_param.deltaRotRad /= m_param.deltaStepFactor;
            m_param.gammaRotU /= m_param.gammaStepFactor;
            m_param.gammaRotL /= m_param.gammaStepFactor;
            m_param.gammaTransU /= m_param.gammaStepFactor;
            m_param.gammaTransL /= m_param.gammaStepFactor;

            if (std::sqrt(m_param.deltaTrans * m_param.deltaTrans + m_param.deltaRotRad * m_param.deltaRotRad) <
                m_param.deltaThresh) {
                break;
            }
        }

        prevDeltaTransform = curDeltaTransform;
        printf("iter: %d, x: %f[m], y: %f[m], z: %f[m], r: %f[deg], p: %f[deg], y: %f[deg], cost: %f, gammaTrans: %f, "
               "gammaRot: %f\n",
               i, curTransform(0), curTransform(1), curTransform(2),
               curTransform(3) * boost::math::double_constants::radian,
               curTransform(4) * boost::math::double_constants::radian,
               curTransform(5) * boost::math::double_constants::radian, prevCost, gammaTrans, gammaRot);
    }

    m_transformation = curTransform;
    return curTransform;
}

template <typename POINT_CLOUD_TYPE>
std::vector<cv::Mat>
CalibrationHandler<POINT_CLOUD_TYPE>::drawPointCloudOnImagePlane(const TransformInfo& transform) const
{
    std::vector<cv::Mat> result;
    result.reserve(m_colorImgs.size());
    for (std::size_t i = 0; i < m_colorImgs.size(); ++i) {
        const auto& colorImg = m_colorImgs[i];
        const auto& curCloud = m_poinclouds[i];
        result.emplace_back(perception::drawPointCloudOnImagePlane<PointCloudType>(colorImg, curCloud, *m_cameraInfo,
                                                                                   perception::toAffine(transform)));
    }

    return result;
}

template <typename POINT_CLOUD_TYPE>
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>
CalibrationHandler<POINT_CLOUD_TYPE>::projectOnPointCloud(const TransformInfo& transform) const
{
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> result;
    result.reserve(m_colorImgs.size());
    for (std::size_t i = 0; i < m_colorImgs.size(); ++i) {
        const auto& colorImg = m_colorImgs[i];
        const auto& curCloud = m_poinclouds[i];
        result.emplace_back(perception::projectOnPointCloud<PointCloudType>(colorImg, curCloud, *m_cameraInfo,
                                                                            perception::toAffine(transform)));
    }

    return result;
}
}  // namespace perception
