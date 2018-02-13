#include "MapMaker.h"

#include <iostream>
#include <set>

#include "Common.h"
#include "MathUtility.h"
#include "Homography.h"

struct MapMakerData
{
    std::set<KeyFrame*> sMeasurementKFs;   // Which keyframes has this map point got measurements in?
    std::set<KeyFrame*> sNeverRetryKFs;    // Which keyframes have measurements failed enough so I should never retry?
    inline int GoodMeasCount()
    {
        return sMeasurementKFs.size();
    }
};

MapMaker::MapMaker(const ATANCamera &cam):
    mCamera(cam)
{
}

bool MapMaker::InitFromStereo(
        KeyFrame &kFirst, KeyFrame &kSecond,
        std::vector<std::pair<cv::Point2i, cv::Point2i> > &vTrailMatches,
        Sophus::SE3 &se3CameraPos)
{
    mdWiggleScale = 0.1;

    mCamera.SetImageSize(cv::Point2f(640,480));

    std::vector<std::pair<cv::Point2i, cv::Point2i> >().swap(vTrailMatches);
    vTrailMatches.clear();

    vTrailMatches.push_back(std::pair<cv::Point2i, cv::Point2i>(cv::Point2i(491.913, 94.5141),cv::Point2i(446.78, 100.857)));
    vTrailMatches.push_back(std::pair<cv::Point2i, cv::Point2i>(cv::Point2i(205.123, 102.715),cv::Point2i(159.491, 109.362)));
    vTrailMatches.push_back(std::pair<cv::Point2i, cv::Point2i>(cv::Point2i(201.812, 309.027),cv::Point2i(157.146, 319.571)));
    vTrailMatches.push_back(std::pair<cv::Point2i, cv::Point2i>(cv::Point2i(506.192, 304.866),cv::Point2i(462.334, 308.977)));

    std::vector<Homography::HomographyMatch> vMatches;
    for(unsigned int i=0; i<vTrailMatches.size(); i++)
    {
        cv::Point2f pt2CamPlaneFirst = mCamera.UnProject(vTrailMatches[i].first);
        cv::Point2f pt2CamPlaneSecond = mCamera.UnProject(vTrailMatches[i].second);
        Eigen::Matrix2d m2PixelProjectionJac = mCamera.GetProjectionDerivs();

        Homography::HomographyMatch match;
        match.v2CamPlaneFirst[0]  = (double)pt2CamPlaneFirst.x;
        match.v2CamPlaneFirst[1]  = (double)pt2CamPlaneFirst.y;
        match.v2CamPlaneSecond[0] = (double)pt2CamPlaneSecond.x;
        match.v2CamPlaneSecond[1] = (double)pt2CamPlaneSecond.y;
        match.m2PixelProjectionJac = m2PixelProjectionJac;
        vMatches.push_back(match);
    }

    Homography homo;
    Sophus::SE3 se3_homo;
    if(GS::RET_FAILED == homo.Compute(vMatches,se3_homo))
        return false;

    std::cout << "se3_homo: \n" << se3_homo.matrix() << std::endl;

    Eigen::Vector3d translation = se3_homo.translation();
    double translation_norm2 = std::sqrt(translation.dot(translation));
    if(translation_norm2 < 1e-10)
    {
        return false;
    }
    se3_homo.translation() = translation/translation_norm2*mdWiggleScale;

    std::cout << "ptW:" << std::endl;
    for(unsigned int i=0; i<vMatches.size(); i++)
    {
        Eigen::Vector3d ptW = cg::MathUtility::Triangulate(se3_homo, vMatches[i].v2CamPlaneSecond, vMatches[i].v2CamPlaneFirst);
        std::cout << "pt: \n" << vMatches[i].v2CamPlaneFirst << ",\n" << ptW << std::endl;
    }

    return true;
}
