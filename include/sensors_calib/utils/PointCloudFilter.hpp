/**
 * @file    PointCloudFilter.hpp
 *
 */

#pragma once

#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>

namespace perception
{
template <typename POINT_CLOUD_TYPE> class PointCloudFilter
{
 public:
    using PointCloudType = POINT_CLOUD_TYPE;
    using PointCloud = pcl::PointCloud<PointCloudType>;
    using PointCloudPtr = typename PointCloud::Ptr;
    using PointCloudConstPtr = typename PointCloud::ConstPtr;

 public:
    static PointCloudPtr filterXAxis(const PointCloudPtr& inPcl, const double minVal, const double maxVal);

    static PointCloudPtr filterYAxis(const PointCloudPtr& inPcl, const double minVal, const double maxVal);

    static PointCloudPtr filterZAxis(const PointCloudPtr& inPcl, const double minVal, const double maxVal);

    static PointCloudPtr filterXYZAxis(const PointCloudPtr& inPcl, const double minXVal, const double maxXVal,
                                       const double minYVal, const double maxYVal, const double minZVal,
                                       const double maxZVal);
};
}  // namespace perception

namespace perception
{
template <typename POINT_CLOUD_TYPE>
typename PointCloudFilter<POINT_CLOUD_TYPE>::PointCloudPtr
PointCloudFilter<POINT_CLOUD_TYPE>::filterXAxis(const PointCloudPtr& inPcl, const double minVal, const double maxVal)
{
    PointCloudPtr filteredPcl(new PointCloud);

    typename pcl::PassThrough<PointCloudType>::Ptr passthroughFilter(new pcl::PassThrough<PointCloudType>);
    passthroughFilter->setInputCloud(inPcl);
    passthroughFilter->setFilterLimits(minVal, maxVal);
    passthroughFilter->setFilterFieldName("x");
    passthroughFilter->filter(*filteredPcl);

    return filteredPcl;
}

template <typename POINT_CLOUD_TYPE>
typename PointCloudFilter<POINT_CLOUD_TYPE>::PointCloudPtr
PointCloudFilter<POINT_CLOUD_TYPE>::filterYAxis(const PointCloudPtr& inPcl, const double minVal, const double maxVal)
{
    PointCloudPtr filteredPcl(new PointCloud);

    typename pcl::PassThrough<PointCloudType>::Ptr passthroughFilter(new pcl::PassThrough<PointCloudType>);
    passthroughFilter->setInputCloud(inPcl);
    passthroughFilter->setFilterLimits(minVal, maxVal);
    passthroughFilter->setFilterFieldName("y");
    passthroughFilter->filter(*filteredPcl);

    return filteredPcl;
}

template <typename POINT_CLOUD_TYPE>
typename PointCloudFilter<POINT_CLOUD_TYPE>::PointCloudPtr
PointCloudFilter<POINT_CLOUD_TYPE>::filterZAxis(const PointCloudPtr& inPcl, const double minVal, const double maxVal)
{
    PointCloudPtr filteredPcl(new PointCloud);

    typename pcl::PassThrough<PointCloudType>::Ptr passthroughFilter(new pcl::PassThrough<PointCloudType>);
    passthroughFilter->setInputCloud(inPcl);
    passthroughFilter->setFilterLimits(minVal, maxVal);
    passthroughFilter->setFilterFieldName("z");
    passthroughFilter->filter(*filteredPcl);

    return filteredPcl;
}

template <typename POINT_CLOUD_TYPE>
typename PointCloudFilter<POINT_CLOUD_TYPE>::PointCloudPtr
PointCloudFilter<POINT_CLOUD_TYPE>::filterXYZAxis(const PointCloudPtr& inPcl, const double minXVal,
                                                  const double maxXVal, const double minYVal, const double maxYVal,
                                                  const double minZVal, const double maxZVal)
{
    PointCloudPtr filteredPcl(new PointCloud);

    typename pcl::CropBox<PointCloudType>::Ptr boxFilter(new pcl::CropBox<PointCloudType>);
    boxFilter->setMin(Eigen::Vector4f(minXVal, minYVal, minZVal, 1.0));
    boxFilter->setMax(Eigen::Vector4f(maxXVal, maxYVal, maxZVal, 1.0));
    boxFilter->setInputCloud(inPcl);
    boxFilter->filter(*filteredPcl);

    return filteredPcl;
}
}  // namespace perception
