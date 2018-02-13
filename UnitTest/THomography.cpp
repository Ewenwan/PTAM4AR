#include "Homography.h"

#include <iostream>
#include <gtest/gtest.h>
#include <Eigen/SVD>

#include "Common.h"
#include "ATANCamera.h"

TEST(Homography,HomographyFromMatches)
{
    std::vector<std::pair<cv::Point2i, cv::Point2i> > vTrailMatches;
    vTrailMatches.push_back(std::pair<cv::Point2i, cv::Point2i>(cv::Point2i(491.913, 94.5141),cv::Point2i(446.78, 100.857)));
    vTrailMatches.push_back(std::pair<cv::Point2i, cv::Point2i>(cv::Point2i(205.123, 102.715),cv::Point2i(159.491, 109.362)));
    vTrailMatches.push_back(std::pair<cv::Point2i, cv::Point2i>(cv::Point2i(201.812, 309.027),cv::Point2i(157.146, 319.571)));
    vTrailMatches.push_back(std::pair<cv::Point2i, cv::Point2i>(cv::Point2i(506.192, 304.866),cv::Point2i(462.334, 308.977)));

    ATANCamera camera;
    camera.SetImageSize(cv::Point2f(640,480));

    std::vector<Homography::HomographyMatch> vMatches;
    for(unsigned int i=0; i<vTrailMatches.size(); i++)
    {
        cv::Point2f pt2CamPlaneFirst = camera.UnProject(vTrailMatches[i].first);
        cv::Point2f pt2CamPlaneSecond = camera.UnProject(vTrailMatches[i].second);
        Eigen::Matrix2d m2PixelProjectionJac = camera.GetProjectionDerivs();

        Homography::HomographyMatch match;
        match.v2CamPlaneFirst[0]  = pt2CamPlaneFirst.x;
        match.v2CamPlaneFirst[1]  = pt2CamPlaneFirst.y;
        match.v2CamPlaneSecond[0] = pt2CamPlaneSecond.x;
        match.v2CamPlaneSecond[1] = pt2CamPlaneSecond.y;
        match.m2PixelProjectionJac = m2PixelProjectionJac;
        vMatches.push_back(match);

        std::cout << "v2CamPlane: \n" << match.v2CamPlaneFirst << ",\n" << match.v2CamPlaneSecond << std::endl;
        std::cout << "m2PixelProjectionJac: \n" <<  match.m2PixelProjectionJac << std::endl;
    }

    Homography homo;
    Eigen::Matrix3d m3Homography;
    if(GS::RET_SUCESS == homo.HomographyFromMatches(vMatches, m3Homography))
    {
        std::cout << "m3Homography: \n" << m3Homography << std::endl;
    }
}
