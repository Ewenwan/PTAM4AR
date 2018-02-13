#include "Common.h"
#include "ImgProc.h"

#include <vector>

bool ImgProc::IsInImageWithBorder(const cv::Mat &image, const cv::Point2i &pt, int borderX, int borderY)
{
    return pt.x >= borderX && pt.y >= borderY && pt.x < image.cols - borderX && pt.y < image.rows - borderY;
}

//Mean Pyramid
int ImgProc::HalfSample(const cv::Mat &imgSrc, cv::Mat &imgDst)
{
    if(1 != imgSrc.channels())
        return GS::RET_FAILED;
    imgDst.create(imgSrc.rows/2,imgSrc.cols/2,CV_8UC1);
    const unsigned char *top = imgSrc.data;
    const unsigned char *bottom = top + imgSrc.cols;
    const unsigned char *end = top + imgSrc.step*imgSrc.rows;
    int ow = imgDst.cols;
    int skip = imgSrc.cols + (imgSrc.cols % 2);
    unsigned char *p = imgDst.data;
    while (bottom < end)
    {
        for (int j=0; j<ow; j++)
        {
            *p = static_cast<unsigned char>((top[0] + top[1] + bottom[0] + bottom[1])/4);
            p++;
            top += 2;
            bottom += 2;
        }
        top += skip;
        bottom += skip;
    }
    return GS::RET_SUCESS;
}

int ImgProc::SumOfPixels(const cv::Mat &img, int &nSum, int &nSumSq)
{
    if(img.empty())
        return GS::RET_FAILED;

    if(1 != img.channels())
        return GS::RET_FAILED;

    nSum = 0;
    nSumSq = 0;
    for(unsigned int h=0; h<img.rows; ++h)
    {
        const unsigned char *pImg = img.ptr<uchar>(h);
        for(unsigned int w=0; w<img.cols; ++w)
        {
            int intensity = static_cast<int>(pImg[w]);
            nSum += intensity;
            nSumSq += intensity*intensity;
        }
    }

    return GS::RET_SUCESS;
}

int ImgProc::SSDAtPoint(const cv::Mat &img, const cv::Point2i &ptPos, const cv::Mat &imgPatch, int &nSSD)
{
    unsigned int patchW = imgPatch.cols;
    unsigned int patchH = imgPatch.rows;

    if(!IsInImageWithBorder(img, ptPos, patchW/2+1, patchH/2+1))
        return GS::RET_FAILED;

    cv::Point2i ptImgBase(ptPos.x-patchW/2, ptPos.y-patchH/2);

    int nSumSqDiff = 0;
    for(unsigned int h=0; h<patchH; ++h)
    {
        const unsigned char *pImage    = img.ptr<uchar>(ptImgBase.y+h);
        const unsigned char *pPatch = imgPatch.ptr<uchar>(h);
        for(unsigned int w=0; w<patchW; ++w)
        {
            int nDiff = pImage[ptImgBase.x+w]-pPatch[w];
            nSumSqDiff += nDiff * nDiff;
        }
    }
    nSSD = nSumSqDiff;
    return GS::RET_SUCESS;
}

