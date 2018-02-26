#ifndef __Homography_H
#define __Homography_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>

class Homography
{
public:
    // Homography matches are 2D-2D matches in a stereo pair, unprojected to the Z=1 plane.
    struct HomographyMatch
    {
        // To be filled in by MapMaker
        Eigen::Vector2d v2CamPlaneFirst;
        Eigen::Vector2d v2CamPlaneSecond;
        Eigen::Matrix2d m2PixelProjectionJac;
    };
    struct HomographyDecomposition
    {
        Eigen::Vector3d v3n;
        Eigen::Vector3d v3Tp;
        Eigen::Matrix3d m3Rp;
        double d;

        // The resolved composition
        Sophus::SE3<double> se3SecondFromFirst;
        int nScore;
    };

public:
    int Compute(const std::vector<HomographyMatch> &vMatches, Sophus::SE3<double> &se3SecondFromFirst, double dMaxPixelError=1.0);

public:
    double mdMaxPixelErrorSquared;
    Eigen::Matrix3d mm3BestHomography;

    int HomographyFromMatches(const std::vector<HomographyMatch> &vMatches, Eigen::Matrix3d &m3Homography);
    int HomographyFromMatchesByMLESAC(const std::vector<HomographyMatch> &vMatches, Eigen::Matrix3d &m3Homography, unsigned short nRansac=300);
    int DecomposeHomography(const Eigen::Matrix3d &m3Homography, std::vector<HomographyDecomposition> &vHomographyDecompositions);
    int ChooseBestDecomposition(
            const std::vector<HomographyMatch> &vMatches,
            const std::vector<HomographyMatch> &vMatchesInliers,
            std::vector<HomographyDecomposition> &vHomographyDecompositions);

    inline double MLESACScore(const Eigen::Matrix3d &m3Homography, const HomographyMatch &match)
    {
        Eigen::Vector2d v2Projected = (m3Homography * match.v2CamPlaneFirst.homogeneous()).hnormalized();
        Eigen::Vector2d v2PixelError = match.m2PixelProjectionJac * (match.v2CamPlaneSecond - v2Projected);
        return v2PixelError.dot(v2PixelError);
    }
    inline double SampsonusError(const Eigen::Matrix3d &m3Essential, Eigen::Vector2d v2A, Eigen::Vector2d v2BDash)
    {
        double dError = v2BDash.homogeneous().dot(m3Essential * v2A.homogeneous());
        Eigen::Vector2d fv3Slice = (m3Essential*v2A.homogeneous()).segment(0,2);
        Eigen::Vector2d fTv3DashSlice = (m3Essential.transpose()*v2BDash.homogeneous()).segment(0,2);
        return (dError*dError / (fv3Slice.dot(fv3Slice)+fTv3DashSlice.dot(fTv3DashSlice)));
    }
};

#endif
