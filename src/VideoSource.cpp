#include "Common.h"
#include "VideoSource.h"

#include <fstream>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgcodecs/imgcodecs_c.h>
#include <opencv2/imgproc/imgproc.hpp>

ImageDataSet::ImageDataSet(const std::string &strDatasetDir,const std::string &strAssociationFilePath):
    mStrDatasetDir(strDatasetDir),
    mStrAssociationFilePath(strAssociationFilePath)
{
}

int ImageDataSet::ReadImagesAssociationFile()
{
    std::ifstream fAssociation;
    fAssociation.open(mStrAssociationFilePath.c_str());
    if(!fAssociation.is_open())
        return GS::RET_FAILED;
    while(!fAssociation.eof())
    {
        std::string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            std::stringstream ss;
            ss << s;
            double t;
            std::string sRGB, sD;
            ss >> t;
            mvTimestamps.push_back(t);
            ss >> sRGB;
            mvstrImageFilenamesRGB.push_back(mStrDatasetDir+"/"+sRGB);
            ss >> t;
            ss >> sD;
            mvstrImageFilenamesD.push_back(mStrDatasetDir+"/"+sD);
        }
    }
    return GS::RET_SUCESS;
}

int ImageDataSet::GetFrameRGBBW(cv::Mat &imgRGB, cv::Mat &imgBW)
{
    static bool isInited=false;
    if(!isInited)
    {
        int ret = ReadImagesAssociationFile();
        if(GS::RET_FAILED == ret)
            return GS::RET_FAILED;
        if(mvstrImageFilenamesRGB.empty())
            return GS::RET_FAILED;
        else if(mvstrImageFilenamesD.size()!=mvstrImageFilenamesRGB.size())
            return GS::RET_FAILED;
        isInited = true;
    }

    static unsigned int index = 0;

    cv::Mat imgBGR = cv::imread(mvstrImageFilenamesRGB[index],CV_LOAD_IMAGE_UNCHANGED);
    if(imgBGR.empty())
        return GS::RET_FAILED;

    cv::cvtColor(imgBGR,imgRGB,CV_BGR2RGB );
    cv::cvtColor(imgBGR,imgBW, CV_BGR2GRAY);

    index++;

    if(index == mvstrImageFilenamesRGB.size())
    {
        index = 0;
    }
    return GS::RET_SUCESS;
}
