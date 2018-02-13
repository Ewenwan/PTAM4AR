#include "Common.h"
#include "ImgProc.h"

int MiniPatch::SampleFromImage(const cv::Mat &img, const cv::Point2i &ptPos)
{
    if(!IsInImageWithBorder(img, ptPos, mSizePatch.width/2+1, mSizePatch.height/2+1))
        return GS::RET_FAILED;

    cv::Rect rectROI;
    rectROI.width = mSizePatch.width;
    rectROI.height = mSizePatch.height;
    rectROI.x = ptPos.x-rectROI.width/2;
    rectROI.y = ptPos.y-rectROI.height/2;
    mImgMiniPatch = img(rectROI);

    return GS::RET_SUCESS;
}

// Find a patch by searching at FAST corners in an input image
// If available, a row-corner LUT is used to speed up search through the
// FAST corners
int MiniPatch::FindPatch(cv::Point2i &ptPos, cv::Mat &img, int nRange, std::vector<cv::Point2i> &vCorners,
                         int nMaxSSD, std::vector<int> *pvRowLUT)
{   
    cv::Point2i ptBBoxTL = ptPos - cv::Point2i(nRange, nRange);
    cv::Point2i ptBBoxBR = ptPos + cv::Point2i(nRange, nRange);
    std::vector<cv::Point2i>::iterator i;
    if(NULL == pvRowLUT)
    {
        for(i = vCorners.begin(); i!=vCorners.end(); i++)
        {
            if(i->y >= ptBBoxTL.y)
                break;
        }
    }
    else
    {
        int nTopRow = ptBBoxTL.y;
        if(nTopRow < 0)
            nTopRow = 0;
        if(nTopRow >= (int)pvRowLUT->size())
            nTopRow = (int)pvRowLUT->size()-1;
        i = vCorners.begin()+(*pvRowLUT)[nTopRow];
    }

    cv::Point2i ptBest;
    int nBestSSD = nMaxSSD + 1;
    for(; i!=vCorners.end(); i++)
    {
        if(i->x < ptBBoxTL.x  || i->x > ptBBoxBR.x)
            continue;
        if(i->y > ptBBoxBR.y)
            break;

        int nSSD = 0;
        if(GS::RET_FAILED == SSDAtPoint(img, *i, mImgMiniPatch, nSSD))
            nSSD = nMaxSSD + 1;
        if(nSSD < nBestSSD)
        {
            ptBest = *i;
            nBestSSD = nSSD;
        }
    }
    if(nBestSSD < nMaxSSD)
    {
        ptPos = ptBest;
        return GS::RET_SUCESS;
    }
    else
        return GS::RET_FAILED;
}












