// Copyright 2008 Isis Innovation Limited
#include "KeyFrame.h"

#include <cvd/vision.h>
#include <cvd/fast_corner.h>

#include "ImageProcess.h"

using namespace CVD;
using namespace std;
using namespace GVars3;

/**
 * @brief Perpares a Keyframe from an image.
 *        1. 生成各级金字塔图像 Generates pyramid levels, 
 *        2. 对每层进行FAST角点检测 does FAST detection, etc.
 *        3. 建立角点查找表，加快查找，对每一行的角点创建查找表LUT 加速查找邻居角点 Generate row look-up-table for the FAST corner points
 * @param im gray scale image
 */
void KeyFrame::MakeKeyFrame_Lite(BasicImage<byte> &im)
{
    aLevels[0].im.resize(im.size());// 初始层
    copy(im, aLevels[0].im);

    for(int i=0; i<LEVELS; i++) {
// 1. 生成金字塔图像，上一层下采样得到下一层图像
        Level &lev = aLevels[i];
        if (i != 0) {
            lev.im.resize(aLevels[i - 1].im.size() / 2);// 尺寸减半
            halfSample(aLevels[i - 1].im, lev.im);// 上一层下采样，得到下一层图像
        }
// 2. FAST角点 检测，对每一层进行 FAST角点 检测
        // .. and detect and store FAST corner points.
        // I use a different threshold on each level; this is a bit of a hack
        // whose aim is to balance the different levels' relative feature densities.
        lev.vCorners.clear();// 每一层对应的 FAST 角点
        lev.vCandidates.clear();
        lev.vMaxCorners.clear();
        if (i == 0)// 第0层 图像
            fast_corner_detect_10(lev.im, lev.vCorners, 10);
        if (i == 1)// 第1层 图像
            fast_corner_detect_10(lev.im, lev.vCorners, 15);
        if (i == 2)// 第2层 图像
            fast_corner_detect_10(lev.im, lev.vCorners, 15);
        if (i == 3)// 第4层 图像
            fast_corner_detect_10(lev.im, lev.vCorners, 10);
// 3. 建立角点查找表，加快查找，对每一行的角点创建查找表LUT 加速查找邻居角点
        // Generate row look-up-table for the FAST corner points: this speeds up
        // finding close-by corner points later on.
        unsigned int v = 0;
        lev.vCornerRowLUT.clear();
        for (int y = 0; y < lev.im.size().y; y++) {
            while (v < lev.vCorners.size() && y > lev.vCorners[v].y)
                v++;
            lev.vCornerRowLUT.push_back(v);// 每一行的查找表
        }
    }
}

/**
 * @brief Fills the rest of the keyframe structure needed by the mapmaker:
 *        FAST nonmax suppression, generation of the list of candidates for further map points,
 *        creation of the relocaliser's SmallBlurryImage.
 */
void KeyFrame::MakeKeyFrame_Rest()
{
    static gvar3<double> gvdCandidateMinSTScore("MapMaker.CandidateMinShiTomasiScore", 70, SILENT);

    for (auto &lev : aLevels) {
        fast_nonmax(lev.im, lev.vCorners, 10, lev.vMaxCorners);
        for (auto i=lev.vMaxCorners.begin(); i!=lev.vMaxCorners.end(); i++) {
            if (!lev.im.in_image_with_border(*i, 10))
                continue;
            double dSTScore = ImageProcess::ShiTomasiScoreAtPoint(lev.im, 3, *i);
            if (dSTScore > *gvdCandidateMinSTScore) {
                Candidate c;
                c.irLevelPos = *i;
                c.dSTScore = dSTScore;
                lev.vCandidates.push_back(c);
            }
        }
    }

    pSBI = new SmallBlurryImage(*this);
    pSBI->MakeJacs(pSBI->mimTemplate, pSBI->mimImageJacs);
}

Vector<3> Level::mvLevelColors[] = {
        makeVector( 1.0, 0.0, 0.0),
        makeVector( 1.0, 1.0, 0.0),
        makeVector( 0.0, 1.0, 0.0),
        makeVector( 0.0, 0.0, 0.7),
        makeVector( 0.0, 0.0, 0.7)
};





