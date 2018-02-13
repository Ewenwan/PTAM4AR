#ifndef __MapPoint_H
#define __MapPoint_H

#include <opencv2/core/types.hpp>

class KeyFrame;
class TrackerData;
class MapMakerData;

struct MapPoint
{
    inline MapPoint()
    {
        bBad = false;
        pTData = NULL;
        pMMData = NULL;
        nMEstimatorOutlierCount = 0;
        nMEstimatorInlierCount = 0;
    }

    bool bBad;
    cv::Point3d v3WorldPos;

    // What pixels should be used to search for this point?
    KeyFrame *pPatchSourceKF; // The KeyFrame the point was originally made in
    int nSourceLevel;         // Pyramid level in source KeyFrame
    cv::Point2i irCenter;   // This is in level-coords in the source pyramid level

    // What follows next is a bunch of intermediate vectors - they all lead up
    // to being able to calculate v3Pixel{Down,Right}_W, which the PatchFinder
    // needs for patch warping!

    cv::Point3d v3Center_NC;             // Unit vector in Source-KF coords pointing at the patch center
    cv::Point3d v3OneDownFromCenter_NC;  // Unit vector in Source-KF coords pointing towards one pixel down of the patch center
    cv::Point3d v3OneRightFromCenter_NC; // Unit vector in Source-KF coords pointing towards one pixel right of the patch center
    cv::Point3d v3Normal_NC;             // Unit vector in Source-KF coords indicating patch normal

    cv::Point3d v3PixelDown_W;           // 3-Vector in World coords corresponding to a one-pixel move down the source image
    cv::Point3d v3PixelRight_W;          // 3-Vector in World coords corresponding to a one-pixel move right the source image
    inline void RefreshPixelVectors()// Calculates above two vectors
    {
    }

    // Info for the Mapmaker (not to be trashed by the tracker:)
    MapMakerData *pMMData;

    // Info for the Tracker (not to be trashed by the MapMaker:)
    TrackerData *pTData;

    // Info provided by the tracker for the mapmaker:
    int nMEstimatorOutlierCount;
    int nMEstimatorInlierCount;
};

#endif
