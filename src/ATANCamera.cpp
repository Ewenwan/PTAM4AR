#include "ATANCamera.h"

ATANCamera::ATANCamera()
{
    mvImageSize[0] = 640.0;
    mvImageSize[1] = 480.0;
    RefreshParams();
}

void ATANCamera::SetImageSize(cv::Vec2f size)
{
    mvImageSize = size;
}

void ATANCamera::RefreshParams()
{
    mvFocal[0] = mvImageSize[0] * mCamParams.fx;
    mvFocal[1] = mvImageSize[1] * mCamParams.fy;
    mvCenter.x = mvImageSize[0] * mCamParams.cx-0.5;
    mvCenter.y = mvImageSize[1] * mCamParams.cy-0.5;
    mdW =  mCamParams.w;

    // One over focal length
    mvInvFocal[0] = 1.0 / mvFocal[0];
    mvInvFocal[1] = 1.0 / mvFocal[1];

    // Some radial distortion parameters
    if(mdW != 0.0)
    {
        md2Tan = 2.0 * std::tan(mdW / 2.0);
        mdOneOver2Tan = 1.0 / md2Tan;
        mdWinv = 1.0 / mdW;
        mdDistortionEnabled = 1.0;
    }
    else
    {
        mdWinv = 0.0;
        md2Tan = 0.0;
        mdDistortionEnabled = 0.0;
    }
}

// Project from the camera z=1 plane to image pixels,
cv::Point2f ATANCamera::Project(const cv::Point2f& vCam)
{
    mvLastCam = vCam;
    mdLastR = std::sqrt(vCam.ddot(vCam));
    mbInvalid = (mdLastR > mdMaxR);
    mdLastFactor = rtrans_factor(mdLastR);
    mdLastDistR = mdLastFactor * mdLastR;
    mvLastDistCam = mdLastFactor * mvLastCam;

    mvLastIm.x = mvCenter.x + mvFocal[0] * mvLastDistCam.x;
    mvLastIm.y = mvCenter.y + mvFocal[1] * mvLastDistCam.y;

    return mvLastIm;
}

// Un-project from image pixel coords to the camera z=1 plane
cv::Point2f ATANCamera::UnProject(const cv::Point2f& v2Im)
{
    mvLastIm = v2Im;
    mvLastDistCam.x = (mvLastIm.x - mvCenter.x) * mvInvFocal[0];
    mvLastDistCam.y = (mvLastIm.y - mvCenter.y) * mvInvFocal[1];
    mdLastDistR = std::sqrt(mvLastDistCam.ddot(mvLastDistCam));
    mdLastR = invrtrans(mdLastDistR);
    double dFactor;
    if(mdLastDistR > 0.01)
        dFactor =  mdLastR / mdLastDistR;
    else
        dFactor = 1.0;
    mdLastFactor = 1.0 / dFactor;
    mvLastCam = dFactor * mvLastDistCam;
    return mvLastCam;
}

Eigen::Matrix2d ATANCamera::GetProjectionDerivs()
{
    // get the derivative of image frame wrt camera z=1 frame at the last computed projection
    // in the form (d im1/d cam1, d im1/d cam2)
    //             (d im2/d cam1, d im2/d cam2)

    double dFracBydx = 0.0;
    double dFracBydy = 0.0;

    const double &k = md2Tan;
    const double &x = mvLastCam.x;
    const double &y = mvLastCam.y;
    double r = mdLastR * mdDistortionEnabled;

    if(r < 0.01)
    {
        dFracBydx = 0.0;
        dFracBydy = 0.0;
    }
    else
    {
        dFracBydx = mdWinv * (k * x) / (r*r*(1 + k*k*r*r)) - x * mdLastFactor / (r*r);
        dFracBydy = mdWinv * (k * y) / (r*r*(1 + k*k*r*r)) - y * mdLastFactor / (r*r);
    }

    Eigen::Matrix2d m2Derivs;
    m2Derivs(0,0) = mvFocal[0] * (dFracBydx * x + mdLastFactor);
    m2Derivs(1,0) = mvFocal[1] * (dFracBydx * y);
    m2Derivs(0,1) = mvFocal[0] * (dFracBydy * x);
    m2Derivs(1,1) = mvFocal[1] * (dFracBydy * y + mdLastFactor);

    return m2Derivs;
}
