#include "Tracker.h"

#include <utility>

#include "JsonConfig.h"
#include "GLWindowPangolin.h"
#include "MapMaker.h"

Tracker::Tracker(JsonConfig *pJsonConfig, GLWindowPangolin *pWindowPangolin, MapMaker &mapmaker):
    mpJsonConfig(pJsonConfig),
    mpPangolinWindow(pWindowPangolin),
    mMapMaker(mapmaker)
{
    mCurrentKF.bFixed = false;
    Reset();
}

void Tracker::TrackFrame(const cv::Mat &imgBW, bool bDraw)
{
    mbDraw = bDraw;
    mCurrentKF.MakeKeyFrame_Lite(imgBW);

    mnFrame++;

    if(mbDraw)
    {
        mpPangolinWindow->DrawPoints2D(mCurrentKF.aLevels[0].vCorners, GS::RGB(1,0,1), 1.0f);
    }

    TrackForInitialMap();
}

void Tracker::TrackForInitialMap()
{
    if(mnInitialStage == TRAIL_TRACKING_NOT_STARTED)
    {
        if(mnFrame==5)
        {
            TrailTracking_Start();
            mnInitialStage = TRAIL_TRACKING_STARTED;
        }
        return;
    }
    if(mnInitialStage == TRAIL_TRACKING_STARTED)
    {
        int nGoodTrails = TrailTracking_Advance();
        if(nGoodTrails < 10)
        {
            Reset();
            return;
        }
        if(mnFrame==12)
        {
            std::vector<std::pair<cv::Point2i, cv::Point2i> > vMatches;
            for(std::list<Trail>::iterator i = mlTrails.begin(); i!=mlTrails.end(); i++)
            {
                vMatches.push_back(std::pair<cv::Point2i, cv::Point2i>(i->ptInitialPos,i->ptCurrentPos));
            }
            mMapMaker.InitFromStereo(mFirstKF, mCurrentKF, vMatches, mse3CamFromWorld);
            mnInitialStage = TRAIL_TRACKING_COMPLETE;
        }
    }
}

void Tracker::TrailTracking_Start()
{
    mCurrentKF.MakeKeyFrame_Rest();
    mFirstKF = mCurrentKF;

    std::vector<std::pair<double,cv::Point2i> > vCornersAndSTScores;
    for(unsigned int i=0; i<mCurrentKF.aLevels[0].vCandidates.size(); i++)
    {
        Candidate &c = mCurrentKF.aLevels[0].vCandidates[i];
        vCornersAndSTScores.push_back(std::pair<double,cv::Point2i>(c.dSTScore, c.ptLevelPos));
    }

    std::stable_sort(vCornersAndSTScores.begin(), vCornersAndSTScores.end(),
                     [](const std::pair<double,cv::Point2i> &a,const std::pair<double,cv::Point2i> &b){return a.first > b.first;});

    int nToAdd = mpJsonConfig->GetInt("Tracker.MaxInitialTrails");
    for(unsigned int i = 0; i<vCornersAndSTScores.size() && nToAdd > 0; i++)
    {
        Trail t;
        t.mPatch.SampleFromImage(mCurrentKF.aLevels[0].im, vCornersAndSTScores[i].second);
        t.ptInitialPos = vCornersAndSTScores[i].second;
        t.ptCurrentPos = t.ptInitialPos;
        mlTrails.push_back(t);
        nToAdd--;
    }

    mPreviousFrameKF = mFirstKF;
}

int Tracker::TrailTracking_Advance()
{
    int nGoodTrails = 0;
    int nMaxSSD = mpJsonConfig->GetInt("Tracker.MiniPatchMaxSSD");
    MiniPatch BackwardsPatch;
    Level &lCurrentFrame = mCurrentKF.aLevels[0];
    Level &lPreviousFrame = mPreviousFrameKF.aLevels[0];

    for(std::list<Trail>::iterator i = mlTrails.begin(); i!=mlTrails.end();)
    {
        std::list<Trail>::iterator next = i;
        next++;
        Trail &trail = *i;
        cv::Point2i ptStart = trail.ptCurrentPos;
        cv::Point2i ptEnd = ptStart;
        int nFound = trail.mPatch.FindPatch(ptEnd, lCurrentFrame.im, 10, lCurrentFrame.vCorners, nMaxSSD);
        if(nFound == GS::RET_SUCESS)
        {
            BackwardsPatch.SampleFromImage(lCurrentFrame.im, ptEnd);
            cv::Point2i ptBackWardsFound = ptEnd;
            nFound = BackwardsPatch.FindPatch(ptBackWardsFound, lPreviousFrame.im, 10, lPreviousFrame.vCorners, nMaxSSD);
            cv::Point2i diffPts = ptBackWardsFound - ptStart;
            if(diffPts.dot(diffPts) > 2)
                nFound = GS::RET_FAILED;
            trail.ptCurrentPos = ptEnd;
            nGoodTrails++;
        }
        if(mbDraw)
        {
            GS::RGB rgbStart, rgbEnd;
            if(nFound == GS::RET_SUCESS)
            {
                rgbStart = GS::RGB(1,1,0);
                rgbEnd   = GS::RGB(1,0,0);
            }
            else
            {
                rgbStart = rgbEnd = GS::RGB(0,1,1);
            }
            mpPangolinWindow->DrawLines(trail.ptInitialPos, rgbStart, trail.ptCurrentPos, rgbEnd, 2.0f);
        }
        if(nFound == GS::RET_FAILED)
        {
            mlTrails.erase(i);
        }
        i = next;
    }

    mPreviousFrameKF = mCurrentKF;
    return nGoodTrails;
}

void Tracker::Reset()
{
    mnFrame = 0;
    mnInitialStage = TRAIL_TRACKING_NOT_STARTED;
    mlTrails.clear();
}
