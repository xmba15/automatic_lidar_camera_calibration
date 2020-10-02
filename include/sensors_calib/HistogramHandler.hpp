/**
 * @file    HistogramHandler.hpp
 *
 * @author  btran
 *
 */

#pragma once

#include <memory>
#include <vector>

#include <opencv2/opencv.hpp>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>

#include "Constants.hpp"
#include "Types.hpp"
#include "utils/utils.hpp"

namespace perception
{
class HistogramHandler
{
 public:
    using Ptr = std::shared_ptr<HistogramHandler>;

    using Histogram = cv::Mat;
    using JointHistogram = cv::Mat;

    explicit HistogramHandler(int numBins);
    ~HistogramHandler();

    template <typename PointCloudType>
    bool update(const std::vector<cv::Mat>& grayImgs,
                const std::vector<typename pcl::PointCloud<PointCloudType>::Ptr>& inClouds,
                const CameraInfo& cameraInfo, const Eigen::Affine3d& affine = Eigen::Affine3d::Identity());

    template <typename PointCloudType>
    bool update(const cv::Mat& grayImgs, const typename pcl::PointCloud<PointCloudType>::Ptr& inClouds,
                const CameraInfo& cameraInfo, const Eigen::Affine3d& affine = Eigen::Affine3d::Identity());

    // image gray, lidar pointcloud intensity, join histogram standard deviation
    std::array<double, 2> calculateStds() const;

    int totalPoints() const
    {
        return m_totalPoints;
    }

    const Histogram& grayHist() const
    {
        return m_grayHist;
    }

    const Histogram& intensityHist() const
    {
        return m_intensityHist;
    }

    const JointHistogram& jointHist() const
    {
        return m_jointHist;
    }

 private:
    bool validateImagePoint(const cv::Mat& img, const cv::Point& point);

 public:
    int m_numBins;
    int m_binFraction;
    int m_graySum;
    int m_intensitySum;
    int m_totalPoints;

    Histogram m_grayHist;
    Histogram m_intensityHist;
    JointHistogram m_jointHist;
};
}  // namespace perception
#include "impl/HistogramHandler.ipp"
