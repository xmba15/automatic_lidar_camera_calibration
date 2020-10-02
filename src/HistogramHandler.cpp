/**
 * @file    HistogramHandler.cpp
 *
 * @author  btran
 *
 */

#include <sensors_calib/HistogramHandler.hpp>
#include <sensors_calib/utils/Utility.hpp>

namespace perception
{
HistogramHandler::HistogramHandler(int numBins)
    : m_numBins(numBins)
    , m_graySum(0)
    , m_intensitySum(0)
    , m_totalPoints(0)
{
    if (m_numBins <= 0 || m_numBins > perception::MAX_BINS) {
        throw std::runtime_error("invalid number of bins");
    }

    m_binFraction = MAX_BINS / m_numBins;
    m_intensityHist = Histogram::zeros(1, m_numBins, CV_64FC1);
    m_grayHist = Histogram::zeros(1, m_numBins, CV_64FC1);
    m_jointHist = JointHistogram::zeros(m_numBins, m_numBins, CV_64FC1);
}

HistogramHandler::~HistogramHandler()
{
}

std::array<double, 2> HistogramHandler::calculateStds() const
{
    if (m_totalPoints == 0) {
        return std::array<double, 2>{0, 0};
    }

    double meanGray = m_graySum / m_totalPoints;
    double meanIntensity = m_intensitySum / m_totalPoints;

    double sigmaGray = 0;
    double sigmaIntensity = 0;

    for (int i = 0; i < m_numBins; ++i) {
        sigmaGray += m_grayHist.at<double>(i) * std::pow((i - meanGray), 2);
        sigmaIntensity += m_intensityHist.at<double>(i) * std::pow((i - meanIntensity), 2);
    }

    sigmaGray /= m_totalPoints;
    sigmaIntensity /= m_totalPoints;

    return std::array<double, 2>{std::sqrt(sigmaGray), std::sqrt(sigmaIntensity)};
}

bool HistogramHandler::validateImagePoint(const cv::Mat& img, const cv::Point& point)
{
    return point.x > 0 && point.x < img.cols && point.y > 0 && point.y < img.rows;
}
}  // namespace perception
