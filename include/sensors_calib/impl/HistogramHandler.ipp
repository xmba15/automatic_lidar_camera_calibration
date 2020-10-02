/**
 * @file    HistogramHandler.ipp
 *
 * @author  btran
 *
 */

namespace perception
{
template <typename PointCloudType>
bool HistogramHandler::update(const std::vector<cv::Mat>& grayImgs,
                              const std::vector<typename pcl::PointCloud<PointCloudType>::Ptr>& inClouds,
                              const CameraInfo& cameraInfo, const Eigen::Affine3d& affine)
{
    if (grayImgs.size() != inClouds.size()) {
        DEBUG_LOG("mismatch number of images and point clouds");
        return false;
    }

    if (grayImgs.empty()) {
        DEBUG_LOG("empty data");
        return false;
    }

    for (std::size_t i = 0; i < grayImgs.size(); ++i) {
        const auto& inCloud = inClouds[i];
        const auto& grayImg = grayImgs[i];

        this->update<PointCloudType>(grayImg, inCloud, cameraInfo, affine);
    }

    return true;
}

template <typename PointCloudType>
bool HistogramHandler::update(const cv::Mat& grayImg, const typename pcl::PointCloud<PointCloudType>::Ptr& inCloud,
                              const CameraInfo& cameraInfo, const Eigen::Affine3d& affine)
{
    if (grayImg.type() != CV_8UC1) {
        DEBUG_LOG("need to use gray image");
        return false;
    }

    typename pcl::PointCloud<PointCloudType>::Ptr alignedCloud(new pcl::PointCloud<PointCloudType>());
    pcl::transformPointCloud(*inCloud, *alignedCloud, affine.matrix());

    for (const auto& point : alignedCloud->points) {
        cv::Point imgPoint = projectToImagePlane(point, cameraInfo);
        if (!this->validateImagePoint(grayImg, imgPoint)) {
            continue;
        }

        int intensityBin = point.intensity / m_binFraction;
        int grayBin = grayImg.ptr<uchar>(imgPoint.y)[imgPoint.x] / m_binFraction;

        m_intensityHist.at<double>(intensityBin)++;
        m_grayHist.at<double>(grayBin)++;
        m_jointHist.at<double>(grayBin, intensityBin)++;

        m_intensitySum += intensityBin;
        m_graySum += grayBin;

        m_totalPoints++;
    }

    return true;
}
}  // namespace perception
