#include "PatchFinder.h"

#include <Eigen/Geometry>

int PatchFinder::MakeTemplateCoarseNoWarp(const KeyFrame &keyframe, unsigned int level, cv::Point2i ptLevelPos)
{
    mnSearchLevel = level;
    cv::Mat img = keyframe.aLevels[level].im;
    if(!IsInImageWithBorder(img, ptLevelPos, mSizePatch.width/2+1, mSizePatch.height/2+1))
    {
        mbTemplateBad = true;
        return GS::RET_FAILED;
    }
    mbTemplateBad = false;

    SampleFromImage(img, ptLevelPos);

    SumOfPixels(mImgMiniPatch, mnTemplateSum, mnTemplateSumSq);

    return GS::RET_SUCESS;
}

// Makes an inverse composition template out of the coarse template.
// Includes calculating image of derivatives (gradients.) The inverse composition
// used here operates on three variables: x offet, y offset, and difference in patch
// means; hence things like mm3HInv are dim 3, but the trivial mean jacobian
// (always unity, for each pixel) is not stored.
void PatchFinder::MakeSubPixTemplate()
{
    //mimJacs.resize(mimTemplate.size() - ImageRef(2,2));
    Eigen::Matrix3d m3H = Eigen::Matrix3d::Zero(3,3); // This stores jTj.
    cv::Point2i pt;
    for(pt.x=1; pt.x<mSizePatch.width-1; pt.x++)
    {
        for(pt.y=1; pt.y<mSizePatch.height-1; pt.y++)
        {
            Eigen::Vector2d v2Grad;
            v2Grad[0] = 0.5 * (mImgMiniPatch.at<uchar>(pt + cv::Point2i(1,0)) - mImgMiniPatch.at<uchar>(pt-cv::Point2i(1,0)));
            v2Grad[1] = 0.5 * (mImgMiniPatch.at<uchar>(pt + cv::Point2i(0,1)) - mImgMiniPatch.at<uchar>(pt-cv::Point2i(0,1)));
            //mimJacs[pt-ImageRef(1,1)].first = v2Grad[0];
            //mimJacs[pt-ImageRef(1,1)].second = v2Grad[1];
            Eigen::Vector3d v3Grad = v2Grad.homogeneous(); // This adds the mean-difference jacobian..
            m3H += v3Grad * v3Grad.transpose(); // Populate JTJ.
        }
    }

    // Invert H = JTJ..
    // Cholesky Decomposition
    mm3HInv = m3H.inverse();// H^-1

    mv2SubPixPos = mv2CoarsePos; // Start the sub-pixel search at the result of the coarse search..
    mdMeanDiff = 0.0;
}

// Iterate inverse composition until convergence. Since it should never have
// to travel more than a pixel's distance, set a max number of iterations;
// if this is exceeded, consider the IC to have failed.
bool PatchFinder::IterateSubPixToConvergence(KeyFrame &kf, int nMaxIts)
{
    const double dConvLimit = 0.03;
    bool bConverged = false;
    int nIts;
    for(nIts = 0; nIts < nMaxIts && !bConverged; nIts++)
    {
        double dUpdateSquared = 0;//IterateSubPix(kf);
        if(dUpdateSquared < 0) // went off edge of image
            return false;
        if(dUpdateSquared < dConvLimit*dConvLimit)
            return true;
    }
    return false;
}
