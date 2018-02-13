#ifndef __PatchFinder_H
#define __PatchFinder_H

#include <Eigen/Core>

#include "Common.h"
#include "ImgProc.h"
#include "KeyFrame.h"
#include "MapPoint.h"

class PatchFinder : public MiniPatch
{
public:
    PatchFinder():MiniPatch(GS::Size(8,8)){}
    ~PatchFinder(){}

    inline int MakeTemplateCoarseNoWarp(const KeyFrame &keyframe, unsigned int level, cv::Point2i ptLevelPos);
    void MakeSubPixTemplate();
    inline void SetSubPixPos(cv::Point2d v2)  { mv2SubPixPos = v2;     }  // Set starting point
    inline bool IterateSubPixToConvergence(KeyFrame &kf, int nMaxIts);

private:
    // Some values stored for the coarse template:
    int mnTemplateSum;    // Cached pixel-sum of the coarse template
    int mnTemplateSumSq;  // Cached pixel-squared sum of the coarse template

    //cv::Mat<std::pair<float,float> > mimJacs;  // Inverse composition jacobians; stored as floats to save a bit of space.

    Eigen::Matrix2d mm2WarpInverse;   // Warping matrix
    int mnSearchLevel;          // Search level in input pyramid
    Eigen::Matrix3d mm3HInv;          // Inverse composition JtJ^-1
    cv::Point2d mv2SubPixPos;     // In the scale of level 0
    double mdMeanDiff;          // Updated during inverse composition

    cv::Point2i mirPredictedPos;  // Search center location of FindPatchCoarse in L0
    cv::Point2d mv2CoarsePos;     // In the scale of level 0; hence the use of vector rather than ImageRef
    cv::Point2i mirCenter;    // Quantized location of the center pixel of the NxN pixel template
    bool mbFound;               // Was the patch found?
    bool mbTemplateBad;         // Error during template generation?

    // Some cached values to avoid duplicating work if the camera is stopped:
    MapPoint *mpLastTemplateMapPoint;  // Which was the last map point this PatchFinder used?
    Eigen::Matrix2d mm2LastWarpMatrix;       // What was the last warp matrix this PatchFinder used?
};

#endif
