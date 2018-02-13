#include "ATANCamera.h"

#include <gtest/gtest.h>

TEST(ATANCamera,UnProject)
{
    ATANCamera camera;
    camera.SetImageSize(cv::Point2f(640,480));
    cv::Point2f v2CamPlaneFirst = camera.UnProject(cv::Point2f(491, 94));
    std::cout << "v2CamPlaneFirst: " << v2CamPlaneFirst.x << "," << v2CamPlaneFirst.y << std::endl;
    std::cout << "mvLastIm: " << camera.mvLastIm.x << "," << camera.mvLastIm.y << std::endl;
    std::cout << "mvCenter: " << camera.mvCenter.x << "," << camera.mvCenter.y << std::endl;
    std::cout << "mvInvFocal: " << camera.mvInvFocal[0] << "," << camera.mvInvFocal[1] << std::endl;
}
