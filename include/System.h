#pragma once
#ifndef __System_H
#define __System_H

#include <opencv2/core/mat.hpp>

class JsonConfig;
class GLWindowPangolin;
class VideoSource;
class ATANCamera;
class MapMaker;
class Tracker;

class System
{
public:
    System();
    ~System();
    void Run();
    void Update(const cv::Mat &imgBW, const cv::Mat &imgRGB);

private:
    JsonConfig *mpJsonConfig;
    GLWindowPangolin *mpPangolinWindow;
    VideoSource *mpVideoSource;
    ATANCamera *mpCamera;
    MapMaker *mpMapMaker;
    Tracker *mpTracker;

    bool mbDone;
};
#endif
