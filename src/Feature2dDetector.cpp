#include "Common.h"
#include "ImgProc.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

int Feature2dDetector::DetectFASTCorners(
        const cv::Mat &srcImage,
        std::vector<cv::Point2i> &vecKeyPoints,
        const int &threshold,
        bool nonmaxSuppression)
{
    if(!srcImage.data)
    {
        return GS::RET_FAILED;
    }
    cv::Mat srcGrayImage;
    if(3 == srcImage.channels())
    {
        cv::cvtColor(srcImage, srcGrayImage, CV_RGB2GRAY);
    }
    else if(1 == srcImage.channels())
    {
        srcImage.copyTo(srcGrayImage);
    }
    std::vector<cv::KeyPoint>  vecKeyPts;
    cv::FAST(srcGrayImage,vecKeyPts,threshold,nonmaxSuppression,cv::FastFeatureDetector::TYPE_9_16);
    unsigned int nSize = vecKeyPts.size();
    vecKeyPoints.resize(nSize);
    for(unsigned int i=0; i<nSize; ++i)
    {
        vecKeyPoints[i] =vecKeyPts[i].pt;
    }
#if defined(DEBUG) || defined(_DEBUG)
    cv::Mat srcGrayImageFASTCorners;
    cv::drawKeypoints(srcGrayImage,vecKeyPts,srcGrayImageFASTCorners,cv::Scalar(0,255,0),cv::DrawMatchesFlags::DEFAULT);
    cv::imwrite("srcGrayImageFASTCorners.bmp",srcGrayImageFASTCorners);
#endif
    return GS::RET_SUCESS;
}

int Feature2dDetector::FindShiTomasiScoreAtPoint(const cv::Mat &image, cv::Point2i ptCenter, double &dScore, unsigned int nHalfBoxSize)
{
    if(!image.data)
    {
        return GS::RET_FAILED;
    }
    if(!IsInImageWithBorder(image,ptCenter,nHalfBoxSize+1,nHalfBoxSize+1))
        return GS::RET_FAILED;

    double dXX = 0;
    double dYY = 0;
    double dXY = 0;

    cv::Point2i ptStart = ptCenter - cv::Point2i(nHalfBoxSize, nHalfBoxSize);
    cv::Point2i ptEnd   = ptCenter + cv::Point2i(nHalfBoxSize, nHalfBoxSize);

    cv::Rect2i rectBox(ptStart,ptEnd);

    cv::Point2i ir;
    for(ir.y = rectBox.y; ir.y<=rectBox.y+rectBox.height; ir.y++)
    {
        for(ir.x = rectBox.x; ir.x<=rectBox.x+rectBox.width; ir.x++)
        {
            double dx = image.at<uchar>(ir + cv::Point2i(1,0)) - image.at<uchar>(ir - cv::Point2i(1,0));
            double dy = image.at<uchar>(ir + cv::Point2i(0,1)) - image.at<uchar>(ir - cv::Point2i(0,1));
            dXX += dx*dx;
            dYY += dy*dy;
            dXY += dx*dy;
        }
    }

    unsigned int nPixels = (rectBox.width+1)*(rectBox.height+1);
    dXX = dXX / (2.0 * nPixels);
    dYY = dYY / (2.0 * nPixels);
    dXY = dXY / (2.0 * nPixels);

    dScore = 0.5 * (dXX + dYY - sqrt( (dXX + dYY) * (dXX + dYY) - 4 * (dXX * dYY - dXY * dXY) ));//smaller eigenvalue

    return GS::RET_SUCESS;
}
