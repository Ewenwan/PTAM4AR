#pragma once
#ifndef __Tracker_H
#define __Tracker_H

#include "ImgProc.h"
#include "KeyFrame.h"

#include <list>
#include <sophus/se3.hpp>

class JsonConfig;
class GLWindowPangolin;
class MapMaker;

struct Trail
{
    MiniPatch mPatch;
    cv::Point2i ptCurrentPos;
    cv::Point2i ptInitialPos;
};

class Tracker
{
public:
    Tracker(JsonConfig *pJsonConfig, GLWindowPangolin *pWindowPangolin, MapMaker &mapmaker);
    ~Tracker(){}

    void TrackFrame(const cv::Mat &imgBW, bool bDraw);

private:
    bool mbDraw;
    int mnFrame;
    KeyFrame mCurrentKF;
    KeyFrame mFirstKF;
    KeyFrame mPreviousFrameKF;
    std::list<Trail> mlTrails;
    enum{TRAIL_TRACKING_NOT_STARTED,TRAIL_TRACKING_STARTED,TRAIL_TRACKING_COMPLETE} mnInitialStage;

    JsonConfig *mpJsonConfig;
    GLWindowPangolin *mpPangolinWindow;
    MapMaker &mMapMaker;
    Sophus::SE3<double> mse3CamFromWorld;

    void TrackForInitialMap();
    void TrailTracking_Start();
    int TrailTracking_Advance();

    void Reset();
};
#endif
