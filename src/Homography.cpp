#include "Homography.h"

#include <iostream>
#include <algorithm>
#include <Eigen/SVD>

#include "Common.h"

int Homography::Compute(const std::vector<HomographyMatch> &vMatches, Sophus::SE3<double> &se3SecondFromFirst, double dMaxPixelError)
{
    mdMaxPixelErrorSquared = dMaxPixelError * dMaxPixelError;

    if(vMatches.empty())
        return GS::RET_FAILED;

    unsigned int nSizeMatches = vMatches.size();

    if(nSizeMatches < 4)
        return GS::RET_FAILED;

    //get mm3BestHomography
    if(nSizeMatches < 10)
    {
        if(GS::RET_FAILED == HomographyFromMatches(vMatches,mm3BestHomography))
            return GS::RET_FAILED;
    }
    else
    {
        if(GS::RET_FAILED == HomographyFromMatchesByMLESAC(vMatches,mm3BestHomography))
            return GS::RET_FAILED;
    }

    std::cout << "mm3BestHomography: \n" << mm3BestHomography << std::endl;

    //get vMatchesInliers
    std::vector<HomographyMatch> vMatchesInliers;
    for(unsigned int i=0; i<nSizeMatches; ++i)
    {
        double dScore = MLESACScore(mm3BestHomography, vMatches[i]);
        if(dScore < mdMaxPixelErrorSquared)
            vMatchesInliers.push_back(vMatches[i]);
    }

    //refine mm3BestHomography With vMatchesInliers



    //decompose mm3BestHomography
    std::vector<HomographyDecomposition> vHomographyDecompositions;
    if(GS::RET_FAILED == DecomposeHomography(mm3BestHomography, vHomographyDecompositions))
        return GS::RET_FAILED;

    if(vHomographyDecompositions.size() != 8)
        return GS::RET_FAILED;

    //get the best one from 8 vHomographyDecompositions
    if(GS::RET_FAILED == ChooseBestDecomposition(vMatches, vMatchesInliers, vHomographyDecompositions))
        return GS::RET_FAILED;

    se3SecondFromFirst = vHomographyDecompositions[0].se3SecondFromFirst;

    return GS::RET_SUCESS;
}

int Homography::HomographyFromMatches(const std::vector<HomographyMatch> &vMatches, Eigen::Matrix3d &m3Homography)
{
    if(vMatches.empty())
        return GS::RET_FAILED;

    unsigned int nSizeMatches = vMatches.size();

    if(nSizeMatches < 4)
        return GS::RET_FAILED;

    int nRows = 2 * nSizeMatches;
    if(nRows < 9)
        nRows = 9;

    Eigen::MatrixXd m2Nx9(nRows,9);
    m2Nx9 = Eigen::MatrixXd::Zero(nRows,9);
    for(unsigned int n=0; n<nSizeMatches; n++)
    {
        double u = vMatches[n].v2CamPlaneSecond[0];
        double v = vMatches[n].v2CamPlaneSecond[1];

        double x = vMatches[n].v2CamPlaneFirst[0];
        double y = vMatches[n].v2CamPlaneFirst[1];

        //x2=Hx1 --> Ax=0
        m2Nx9(n*2+0,0) = x;
        m2Nx9(n*2+0,1) = y;
        m2Nx9(n*2+0,2) = 1;
        m2Nx9(n*2+0,3) = 0;
        m2Nx9(n*2+0,4) = 0;
        m2Nx9(n*2+0,5) = 0;
        m2Nx9(n*2+0,6) = -x*u;
        m2Nx9(n*2+0,7) = -y*u;
        m2Nx9(n*2+0,8) = -u;

        m2Nx9(n*2+1,0) = 0;
        m2Nx9(n*2+1,1) = 0;
        m2Nx9(n*2+1,2) = 0;
        m2Nx9(n*2+1,3) = x;
        m2Nx9(n*2+1,4) = y;
        m2Nx9(n*2+1,5) = 1;
        m2Nx9(n*2+1,6) = -x*v;
        m2Nx9(n*2+1,7) = -y*v;
        m2Nx9(n*2+1,8) = -v;
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(m2Nx9, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd m9V = svd.matrixV();
    Eigen::VectorXd v9V = m9V.col(8);//column vector corresponding to the smallest singular value

    m3Homography.row(0) = v9V.segment(0,3);
    m3Homography.row(1) = v9V.segment(3,3);
    m3Homography.row(2) = v9V.segment(6,3);

    return GS::RET_SUCESS;
}

int Homography::HomographyFromMatchesByMLESAC(const std::vector<HomographyMatch> &vMatches, Eigen::Matrix3d &m3Homography, unsigned short nRansac)
{
    if(vMatches.empty() || 0==nRansac)
        return GS::RET_FAILED;

    unsigned int nSizeMatches = vMatches.size();

    if(nSizeMatches < 4)
        return GS::RET_FAILED;

    double dErrorBest = 999999999999999999.9;
    unsigned int indices[4]={0};
    for(unsigned short n=0; n<nRansac; ++n)
    {
        //find 4 unique matches
        for(unsigned char i=0; i<4; i++)
        {
            unsigned int n;
            bool isUnique = false;
            while(!isUnique)
            {
                n = std::rand()%nSizeMatches;
                isUnique = true;
                for(unsigned char j=0; j<i; ++j)
                {
                    if(indices[j] == n)
                        isUnique = false;
                }
            }
            indices[i] = n;
        }
        std::vector<HomographyMatch> vMatchesMini;
        for(unsigned char i=0; i<4; ++i)
        {
            vMatchesMini.push_back(vMatches[indices[i]]);
        }

        Eigen::Matrix3d m3Homo;
        if(GS::RET_SUCESS == HomographyFromMatches(vMatchesMini,m3Homo))
        {
            double dErrorSum = 0.0;
            for(unsigned int i=0;i<nSizeMatches;++i)
            {
                double dErrorSquared = MLESACScore(m3Homography, vMatches[i]);
                dErrorSum += (dErrorSquared > mdMaxPixelErrorSquared) ? mdMaxPixelErrorSquared : dErrorSquared;
            }
            if(dErrorSum < dErrorBest)
            {
                dErrorBest = dErrorSum;
                m3Homography = m3Homo;
            }
        }
        else
        {
            continue;
        }
    }

    return GS::RET_SUCESS;
}

int Homography::DecomposeHomography(const Eigen::Matrix3d &m3Homography, std::vector<HomographyDecomposition> &vHomographyDecompositions)
{
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(m3Homography, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Vector3d v3Diag = svd.singularValues();
    double d1 = fabs(v3Diag[0]);
    double d2 = fabs(v3Diag[1]);
    double d3 = fabs(v3Diag[2]);

    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    double s = U.determinant() * V.determinant();

    double dPrime_PM = d2;

    int nCase;
    if(d1 != d2 && d2 != d3)
        nCase = 1;
    else if( d1 == d2 && d2 == d3)
        nCase = 3;
    else
        nCase = 2;

    if(nCase != 1)
    {
        //Homographyinit: This motion case is not implemented or is degenerate. Try again
        return GS::RET_FAILED;
    }

    double x1_PM;
    double x2;
    double x3_PM;

    // All below deals with the case = 1 case.
    // Case 1 implies (d1 != d3)
    { // Eq. 12
        x1_PM = sqrt((d1*d1 - d2*d2) / (d1*d1 - d3*d3));
        x2    = 0;
        x3_PM = sqrt((d2*d2 - d3*d3) / (d1*d1 - d3*d3));
    };

    double e1[4] = {1.0, -1.0,  1.0, -1.0};
    double e3[4] = {1.0,  1.0, -1.0, -1.0};

    Eigen::Vector3d v3np;
    HomographyDecomposition decomposition;

    // Case 1, d' > 0:
    decomposition.d = s * dPrime_PM;
    for(int signs=0; signs<4; signs++)
    {
        // Eq 13
        decomposition.m3Rp = Eigen::Matrix3d::Identity(3,3);
        double dSinTheta = (d1 - d3) * x1_PM * x3_PM * e1[signs] * e3[signs] / d2;
        double dCosTheta = (d1 * x3_PM * x3_PM + d3 * x1_PM * x1_PM) / d2;
        decomposition.m3Rp(0,0) =  dCosTheta;
        decomposition.m3Rp(0,2) = -dSinTheta;
        decomposition.m3Rp(2,0) =  dSinTheta;
        decomposition.m3Rp(2,2) =  dCosTheta;

        // Eq 14
        decomposition.v3Tp[0] = (d1 - d3) * x1_PM * e1[signs];
        decomposition.v3Tp[1] = 0.0;
        decomposition.v3Tp[2] = (d1 - d3) * -x3_PM * e3[signs];

        v3np[0] = x1_PM * e1[signs];
        v3np[1] = x2;
        v3np[2] = x3_PM * e3[signs];
        decomposition.v3n = V * v3np;

        vHomographyDecompositions.push_back(decomposition);
    }
    // Case 1, d' < 0:
    decomposition.d = s * -dPrime_PM;
    for(int signs=0; signs<4; signs++)
    {
        // Eq 15
        decomposition.m3Rp = -1 * Eigen::Matrix3d::Identity(3,3);
        double dSinPhi = (d1 + d3) * x1_PM * x3_PM * e1[signs] * e3[signs] / d2;
        double dCosPhi = (d3 * x1_PM * x1_PM - d1 * x3_PM * x3_PM) / d2;
        decomposition.m3Rp(0,0) =  dCosPhi;
        decomposition.m3Rp(0,2) =  dSinPhi;
        decomposition.m3Rp(2,0) =  dSinPhi;
        decomposition.m3Rp(2,2) = -dCosPhi;

        // Eq 16
        decomposition.v3Tp[0] = (d1 + d3) * x1_PM * e1[signs];
        decomposition.v3Tp[1] = 0.0;
        decomposition.v3Tp[2] = (d1 + d3) * x3_PM * e3[signs];

        v3np[0] = x1_PM * e1[signs];
        v3np[1] = x2;
        v3np[2] = x3_PM * e3[signs];
        decomposition.v3n = V * v3np;

        vHomographyDecompositions.push_back(decomposition);
    }

    // While we have the SVD results calculated here, store the decomposition R and t results as well..
    for(unsigned int i=0; i<vHomographyDecompositions.size(); i++)
    {
        Eigen::Matrix3d m3Rotation = s * U * vHomographyDecompositions[i].m3Rp * V.transpose();
        Eigen::Vector3d v3Translation =  U * vHomographyDecompositions[i].v3Tp;

        Sophus::SE3<double> se3SecondFromFirst;
        se3SecondFromFirst.setRotationMatrix(m3Rotation);
        se3SecondFromFirst.translation() = v3Translation;

        vHomographyDecompositions[i].se3SecondFromFirst = se3SecondFromFirst;
    }

    return GS::RET_SUCESS;
}

int Homography::ChooseBestDecomposition(
        const std::vector<HomographyMatch> &vMatches,
        const std::vector<HomographyMatch> &vMatchesInliers,
        std::vector<HomographyDecomposition> &vHomographyDecompositions)
{
    if(8 != vHomographyDecompositions.size())
        return GS::RET_FAILED;

    for(unsigned char i=0; i<vHomographyDecompositions.size(); ++i)
    {
        HomographyDecomposition &decom = vHomographyDecompositions[i];
        int nPositive = 0;
        for(unsigned int m=0; m<vMatchesInliers.size(); ++m)
        {
            Eigen::Vector2d v2 = vMatchesInliers[m].v2CamPlaneFirst;
            double dVisibilityTest = (mm3BestHomography(2,0) * v2[0] + mm3BestHomography(2,1) * v2[1] + mm3BestHomography(2,2)) / decom.d;
            if(dVisibilityTest > 0.0)
                nPositive++;
        };
        decom.nScore = -nPositive;
    }

    std::stable_sort(vHomographyDecompositions.begin(), vHomographyDecompositions.end(),
                     [](const HomographyDecomposition &a,const HomographyDecomposition &b){return a.nScore < b.nScore;});
    vHomographyDecompositions.resize(4);

    for(unsigned char i=0; i<vHomographyDecompositions.size(); ++i)
    {
        HomographyDecomposition &decom = vHomographyDecompositions[i];
        int nPositive = 0;
        for(unsigned int m=0; m<vMatchesInliers.size(); ++m)
        {
            Eigen::Vector3d v3 = vMatchesInliers[m].v2CamPlaneFirst.homogeneous();
            double dVisibilityTest = v3.dot(decom.v3n) / decom.d;
            if(dVisibilityTest > 0.0)
                nPositive++;
        };
        decom.nScore = -nPositive;
    }

    std::stable_sort(vHomographyDecompositions.begin(), vHomographyDecompositions.end(),
                     [](const HomographyDecomposition &a,const HomographyDecomposition &b){return a.nScore < b.nScore;});
    vHomographyDecompositions.resize(2);

    // According to Faugeras and Lustman, ambiguity exists if the two scores are equal
    // but in practive, better to look at the ratio!
    double dRatio = vHomographyDecompositions[1].nScore / (double)vHomographyDecompositions[0].nScore;

    if(dRatio < 0.9)
    {
        // no ambiguity!
        vHomographyDecompositions.erase(vHomographyDecompositions.begin() + 1);
    }
    else// two-way ambiguity. Resolve by sampsonus score of all points.
    {
        double dErrorSquaredLimit  = mdMaxPixelErrorSquared * 4;
        double adSampsonusScores[2];
        for(int i=0; i<2; i++)
        {
            Sophus::SE3<double> se3 = vHomographyDecompositions[i].se3SecondFromFirst;
            Eigen::Matrix3d m3Essential;
            for(int j=0; j<3; j++)
            {
                m3Essential.col(j) = se3.translation().cross(se3.rotationMatrix().col(j));
            }
            double dSumError = 0;
            for(unsigned int m=0; m<vMatches.size(); ++m)
            {
                double d = SampsonusError(m3Essential, vMatches[m].v2CamPlaneFirst, vMatches[m].v2CamPlaneSecond);
                if(d > dErrorSquaredLimit)
                    d = dErrorSquaredLimit;
                dSumError += d;
            }

            adSampsonusScores[i] = dSumError;
        }

        if(adSampsonusScores[0] <= adSampsonusScores[1])
            vHomographyDecompositions.erase(vHomographyDecompositions.begin() + 1);
        else
            vHomographyDecompositions.erase(vHomographyDecompositions.begin());
    }
    return GS::RET_SUCESS;
}
