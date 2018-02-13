#include "Common.h"
#include "KeyFrame.h"
#include "ImgProc.h"

#include <assert.h>

int KeyFrame::MakeKeyFrame_Lite(const cv::Mat &img)
{
    if(NULL == img.data)
        return GS::RET_FAILED;
    if(1 != img.channels())
        return GS::RET_FAILED;

    aLevels[0].im = img.clone();
    for(unsigned int i=0;i<LEVELS;++i)
    {
        Level &lev = aLevels[i];
        if(0 != i)
        {
            assert(GS::RET_SUCESS == ImgProc::HalfSample(aLevels[i-1].im,lev.im));
        }
        lev.vCorners.clear();
        lev.vCandidates.clear();
        lev.vMaxCorners.clear();
        switch(i)
        {
        case 0:
            Feature2dDetector::DetectFASTCorners(lev.im, lev.vCorners, 10, false);
            break;
        case 1:
            Feature2dDetector::DetectFASTCorners(lev.im, lev.vCorners, 15, false);
            break;
        case 2:
            Feature2dDetector::DetectFASTCorners(lev.im, lev.vCorners, 15, false);
            break;
        case 3:
            Feature2dDetector::DetectFASTCorners(lev.im, lev.vCorners, 10, false);
            break;
        }
        //generate row LUT
        unsigned int v=0;
        lev.vCornerRowLUT.clear();
        for(int y=0; y<lev.im.rows; y++)
        {
            while(v < lev.vCorners.size() && y > lev.vCorners[v].y)
                v++;
            lev.vCornerRowLUT.push_back(v);
        }
    }
    return GS::RET_SUCESS;
}

int KeyFrame::MakeKeyFrame_Rest()
{
    for(unsigned int l=0; l<LEVELS; ++l)
    {
        Level &lev = aLevels[l];
        Feature2dDetector::DetectFASTCorners(lev.im, lev.vMaxCorners, 10, true);
        for(std::vector<cv::Point2i>::iterator i=lev.vMaxCorners.begin(); i!=lev.vMaxCorners.end(); i++)
        {
            if(!ImgProc::IsInImageWithBorder(lev.im,*i, 10, 10))
                continue;
            double dSTScore = 0.0;
            Feature2dDetector::FindShiTomasiScoreAtPoint(lev.im,*i,dSTScore,3);
            if(dSTScore > 70)
            {
                Candidate c;
                c.ptLevelPos = *i;
                c.dSTScore = dSTScore;
                lev.vCandidates.push_back(c);
            }
        }
    }
    return GS::RET_SUCESS;
}
