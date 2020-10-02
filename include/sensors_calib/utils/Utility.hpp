/**
 * @file    Utility.hpp
 *
 * @author  btran
 *
 */

#pragma once

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>

#include "../Types.hpp"
#include "PointCloudFilter.hpp"

namespace perception
{
inline cv::Scalar colorCodingReflectivityBGR(const int intensity)
{
    std::uint8_t r, g, b;
    if (intensity < 30) {
        r = 0;
        g = static_cast<int>(intensity * 255 / 30) & 0xff;
        b = 255;
    } else if (intensity < 90) {
        r = 0;
        g = 0xff;
        b = static_cast<int>((90 - intensity) * 255 / 60) & 0xff;
    } else if (intensity < 150) {
        r = static_cast<int>((intensity - 90) * 255 / 60) & 0xff;
        g = 0xff;
        b = 0;
    } else {
        r = 0xff;
        g = static_cast<int>((255 - intensity) * 255 / (256 - 150)) & 0xff;
        b = 0;
    }

    return cv::Scalar(b, g, r);
}

TransformInfo getTransformInfo(const std::string transformationInfoPath);

template <typename PointCloudType>
cv::Point projectToImagePlane(const PointCloudType& point3d, const CameraInfo& cameraInfo)
{
    Eigen::Matrix<double, 3, 1> point(point3d.x, point3d.y, point3d.z);
    Eigen::Matrix<double, 3, 1> point2d = cameraInfo.K() * point;  // homogenous coordinate of 2d point

    return cv::Point(point2d(0) / point2d(2), point2d(1) / point2d(2));
}

template <typename PointCloudType>
cv::Mat drawPointCloudOnImagePlane(const cv::Mat& img, const typename pcl::PointCloud<PointCloudType>::Ptr& inCloud,
                                   const CameraInfo& cameraInfo,
                                   const Eigen::Affine3d& affine = Eigen::Affine3d::Identity())
{
    typename pcl::PointCloud<PointCloudType>::Ptr alignedCloud(new pcl::PointCloud<PointCloudType>());
    pcl::transformPointCloud(*inCloud, *alignedCloud, affine.matrix());

    cv::Mat visualizedImg = img.clone();
    for (const auto& point : alignedCloud->points) {
        if (isnan(point.x) || isnan(point.y) || isnan(point.z)) {
            continue;
        }

        cv::Point imgPoint = projectToImagePlane<PointCloudType>(point, cameraInfo);
        if (imgPoint.x < 0 || imgPoint.x >= img.cols || imgPoint.y < 0 || imgPoint.y >= img.rows) {
            continue;
        }

        cv::circle(visualizedImg, imgPoint, 1 /* radius */, colorCodingReflectivityBGR(point.intensity), -1);
    }

    return visualizedImg;
}

template <typename PointCloudType>
pcl::PointCloud<pcl::PointXYZRGB>::Ptr
projectOnPointCloud(const cv::Mat& img, const typename pcl::PointCloud<PointCloudType>::Ptr& inCloud,
                    const CameraInfo& cameraInfo, const Eigen::Affine3d& affine = Eigen::Affine3d::Identity())
{
    typename pcl::PointCloud<PointCloudType>::Ptr alignedCloud(new pcl::PointCloud<PointCloudType>());
    pcl::transformPointCloud(*inCloud, *alignedCloud, affine.matrix());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (std::size_t i = 0; i < inCloud->points.size(); ++i) {
        const auto& origPoint = inCloud->points[i];
        const auto& alignedPoint = alignedCloud->points[i];
        if (isnan(alignedPoint.x) || isnan(alignedPoint.y) || isnan(alignedPoint.z)) {
            continue;
        }
        cv::Point imgPoint = projectToImagePlane<PointCloudType>(alignedPoint, cameraInfo);
        if (imgPoint.x < 0 || imgPoint.x >= img.cols || imgPoint.y < 0 || imgPoint.y >= img.rows) {
            continue;
        }

        pcl::PointXYZRGB outPoint;
        outPoint.x = origPoint.x;
        outPoint.y = origPoint.y;
        outPoint.z = origPoint.z;
        const auto& color = img.ptr<cv::Vec3b>(imgPoint.y)[imgPoint.x];
        outPoint.b = color[0];
        outPoint.g = color[1];
        outPoint.r = color[2];
        outCloud->points.emplace_back(outPoint);
    }

    outCloud->height = 1;
    outCloud->width = outCloud->points.size();
    return outCloud;
}

inline std::vector<std::string> splitByDelim(const std::string& s, const char delimiter)
{
    std::stringstream ss(s);
    std::string token;
    std::vector<std::string> tokens;
    while (std::getline(ss, token, delimiter)) {
        tokens.emplace_back(token);
    }
    return tokens;
}

inline std::vector<std::string> parseMetaDataFile(const std::string& metaDataFilePath)
{
    std::ifstream inFile;
    inFile.open(metaDataFilePath);

    if (!inFile) {
        throw std::runtime_error("unable to open " + metaDataFilePath + "\n");
    }

    std::stringstream buffer;
    buffer << inFile.rdbuf();

    return splitByDelim(buffer.str(), '\n');
}

template <typename T>
bool almostEquals(const T val, const T correctVal, const T epsilon = std::numeric_limits<T>::epsilon())
{
    const T maxXYOne = std::max({static_cast<T>(1.0f), std::fabs(val), std::fabs(correctVal)});
    return std::fabs(val - correctVal) <= epsilon * maxXYOne;
}
}  // namespace perception

#ifdef DEBUG
#define ENABLE_DEBUG 1
#include <iostream>
#else
#define ENABLE_DEBUG 0
#endif

#if ENABLE_DEBUG
#define DEBUG_LOG(...)                                                                                                 \
    {                                                                                                                  \
        char str[200];                                                                                                 \
        snprintf(str, sizeof(str), __VA_ARGS__);                                                                       \
        std::cout << "[" << __FILE__ << "][" << __FUNCTION__ << "][Line " << __LINE__ << "] >>> " << str << std::endl; \
    }
#else
#define DEBUG_LOG(...)
#endif
