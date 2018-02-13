#pragma once
#ifndef __VideoSource_H
#define __VideoSource_H

#include <vector>
#include <string>
#include <sstream>
#include <opencv2/core/mat.hpp>

class VideoSource
{
public:
    unsigned int mWidth;
    unsigned int mHeight;
    VideoSource(){}
    virtual ~VideoSource(){}
    virtual int GetFrameRGBBW(cv::Mat &imgRGB,cv::Mat &imgBW){return 0;}

};

class ImageDataSet : public VideoSource
{
public:
    ImageDataSet(const std::string &strDatasetDir,const std::string &strAssociationFilePath);
    ~ImageDataSet(){}
    int ReadImagesAssociationFile();
    int GetFrameRGBBW(cv::Mat &imgRGB, cv::Mat &imgBW);

private:
    std::string mStrDatasetDir;
    std::string mStrAssociationFilePath;
    std::vector<std::string> mvstrImageFilenamesRGB;
    std::vector<std::string> mvstrImageFilenamesD;
    std::vector<double> mvTimestamps;
};
#endif
