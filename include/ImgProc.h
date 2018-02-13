#pragma once
#ifndef __ImgProc_H
#define __ImgProc_H

#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>

#include "Common.h"

class ImgProc
{
public:
    static bool IsInImageWithBorder(const cv::Mat &image, const cv::Point2i &pt, int borderX=10, int borderY=10);
    static int HalfSample(const cv::Mat &imgSrc, cv::Mat &imgDst);
    int SumOfPixels(const cv::Mat &img, int &nSum, int &nSumSq);
    int SSDAtPoint(const cv::Mat &img, const cv::Point2i &ptPos, const cv::Mat &imgPatch, int &nSSD);
};

class Feature2dDetector : public ImgProc
{
public:
    static int DetectFASTCorners(const cv::Mat &srcImage, std::vector<cv::Point2i> &vecKeyPoints, const int &threshold, bool nonmaxSuppression);
    static int FindShiTomasiScoreAtPoint(const cv::Mat &image, cv::Point2i ptCenter, double &dScore, unsigned int nHalfBoxSize=3);
};

// This is a simple pixel-patch class, used for tracking small patches
// it's used by the tracker for building the initial map
class MiniPatch : public ImgProc
{
public:
    MiniPatch():mSizePatch(GS::Size(9,9)){}
    MiniPatch(GS::Size size):mSizePatch(size){}
    cv::Mat mImgMiniPatch;//Original pixels

    int SampleFromImage(const cv::Mat &img, const cv::Point2i &ptPos);
    int FindPatch(cv::Point2i &ptPos, cv::Mat &img, int nRange, std::vector<cv::Point2i> &vCorners,
                  int nMaxSSD = 9999, std::vector<int> *pvRowLUT = NULL);

protected:
    GS::Size mSizePatch;
};

#endif
