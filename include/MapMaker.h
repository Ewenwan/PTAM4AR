#ifndef __MapMaker_H
#define __MapMaker_H

#include <sophus/se3.h>

#include "ATANCamera.h"
#include "KeyFrame.h"

class MapMaker
{
public:
    MapMaker(const ATANCamera &cam);
    ~MapMaker() {}

    bool InitFromStereo(KeyFrame &kFirst,
                        KeyFrame &kSecond,
                        std::vector<std::pair<cv::Point2i, cv::Point2i> > &vTrailMatches,
                        Sophus::SE3 &se3CameraPos);

private:
    ATANCamera mCamera;
    double mdWiggleScale;
};

#endif
