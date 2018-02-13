#pragma once
#ifndef __ATANCamera_H
#define __ATANCamera_H

#include <cmath>
#include <opencv2/core/types.hpp>
#include <opencv2/core/matx.hpp>
#include <Eigen/Core>

class ATANCamera
{
public:
    struct CamParams
    {
        inline CamParams()
        {
            fx = 0.809762;
            fy = 1.41527;
            cx = 0.501900;
            cy = 0.43248;
            w  = 0.007244;
        }

        float fx;
        float fy;
        float cx;
        float cy;
        float w ;
    }mCamParams;

public:
    ATANCamera();
    ~ATANCamera() {}
    cv::Point2f Project(const cv::Point2f &vCam);
    cv::Point2f UnProject(const cv::Point2f& v2Im);
    Eigen::Matrix2d GetProjectionDerivs();
    void SetImageSize(cv::Vec2f size);
    void RefreshParams() ;

public:
//protected:
    // Cached from the last project/unproject:
    cv::Point2f mvLastCam;      // Last z=1 coord
    cv::Point2f mvLastIm;       // Last image/UFB coord
    cv::Point2f mvLastDistCam;  // Last distorted z=1 coord
    double mdLastR;           // Last z=1 radius
    double mdLastDistR;       // Last z=1 distorted radius
    double mdLastFactor;      // Last ratio of z=1 radii
    bool mbInvalid;           // Was the last projection invalid?

    // Cached from last RefreshParams:
    double mdLargestRadius; // Largest R in the image
    double mdMaxR;          // Largest R for which we consider projection valid
    double mdOnePixelDist;  // z=1 distance covered by a single pixel offset (a rough estimate!)
    double md2Tan;          // distortion model coeff
    double mdOneOver2Tan;   // distortion model coeff
    double mdW;             // distortion model coeff
    double mdWinv;          // distortion model coeff
    double mdDistortionEnabled; // One or zero depending on if distortion is on or off.
    cv::Point2f mvCenter;     // Pixel projection center
    cv::Vec2f mvFocal;      // Pixel focal length
    cv::Vec2f mvInvFocal;   // Inverse pixel focal length
    cv::Vec2f mvImageSize;
    cv::Vec2f mvUFBLinearFocal;
    cv::Vec2f mvUFBLinearInvFocal;
    cv::Vec2f mvUFBLinearCenter;
    cv::Vec2f mvImplaneTL;
    cv::Vec2f mvImplaneBR;

    // Radial distortion transformation factor: returns ration of distorted / undistorted radius.
    inline double rtrans_factor(double r)
    {
        if(r < 0.001 || mdW == 0.0)
            return 1.0;
        else
            return (mdWinv* std::atan(r * md2Tan) / r);
    }

    // Inverse radial distortion: returns un-distorted radius from distorted.
    inline double invrtrans(double r)
    {
        if(mdW == 0.0)
            return r;
        return(std::tan(r * mdW) * mdOneOver2Tan);
    }
};

#endif
