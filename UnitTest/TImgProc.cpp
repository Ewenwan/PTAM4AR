#include "Common.h"
#include "ImgProc.h"

#include <iostream>
#include <fstream>
#include <iomanip>
#include <gtest/gtest.h>

#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

cv::Mat srcImage = cv::imread("/home/gordon/projects/PTAM4AR/data/rgbd_dataset_freiburg1_xyz/rgb/1305031102.343233.png");

TEST(ImgProc,HalfSample)
{
    cv::Mat dstImg;
    cv::Mat imgBW;
    cv::cvtColor(srcImage,imgBW, CV_BGR2GRAY);
    ASSERT_EQ(ImgProc::HalfSample(imgBW,dstImg), GS::RET_SUCESS);
    cv::imwrite("HalfSample.bmp",dstImg);
    ASSERT_EQ(imgBW.cols/2, dstImg.cols);
    ASSERT_EQ(imgBW.rows/2, dstImg.rows);
    const unsigned char *srcT = imgBW.data;
    const unsigned char *srcB = imgBW.data+imgBW.cols;
    const unsigned char *dstT = dstImg.data;
    ASSERT_EQ((srcT[0]+srcT[1]+srcB[0]+srcB[1])/4, dstT[0]);
}

TEST(Feature2dDetector,DetectFASTCorners)
{
    std::vector<cv::Point2i> vecKeyPoints;
    int ret = Feature2dDetector::DetectFASTCorners(srcImage, vecKeyPoints, 10, true);
    ASSERT_EQ(ret, GS::RET_SUCESS);
}

TEST(Feature2dDetector,FindShiTomasiScoreAtPoint)
{
    cv::Mat imgBW;
    cv::cvtColor(srcImage,imgBW, CV_BGR2GRAY);
    double score = 0;
    int ret = Feature2dDetector::FindShiTomasiScoreAtPoint(imgBW, cv::Point2i(215,278), score, 3);
    ASSERT_EQ(ret, GS::RET_SUCESS);
    std::cout << "score: " << score << std::endl;
}

TEST(MiniPatch,SSDAtPoint)
{
    //desktop log.log
    cv::Mat imgBW1,imgBW2;
    cv::cvtColor(srcImage,imgBW1, CV_BGR2GRAY);

    cv::Mat srcImage2 = cv::imread("/home/gordon/projects/PTAM4AR/data/rgbd_dataset_freiburg1_xyz/rgb/1305031102.375329.png");
    cv::cvtColor(srcImage2,imgBW2, CV_BGR2GRAY);

    MiniPatch miniPatch;
    ASSERT_EQ(GS::RET_SUCESS,miniPatch.SampleFromImage(imgBW1,cv::Point2i(147,346)));
    int nSSD = 0;
    miniPatch.SSDAtPoint(imgBW2, cv::Point2i(137,336), miniPatch.mImgMiniPatch, nSSD);
    std::cout << "nSSD: " << nSSD << std::endl;
    //ASSERT_EQ(nSSD,518389);

    std::ofstream outfile("outfileImgTEST.txt");
    cv::Mat matOrig = miniPatch.mImgMiniPatch;
    for(int h=0; h<matOrig.rows; h++)
    {
        unsigned char *pData = matOrig.ptr<uchar>(h);
        for(int w=0; w<matOrig.cols; w++)
        {
            unsigned char data = pData[w];
            outfile << std::setfill('0') << std::setw(3) << (int)data << " ";
        }
        outfile<<std::endl;
    }
    outfile.close();

//    std::ofstream outfile("imgBW1TEST.txt");
//    cv::Mat matOrig = imgBW1;
//    for(int h=100; h<130; h++)
//    {
//        unsigned char *pData = matOrig.ptr<uchar>(h);
//        for(int w=100; w<130; w++)
//        {
//            unsigned char data = pData[w];
//            outfile << std::setfill('0') << std::setw(3) << (int)data << " ";
//        }
//        outfile<<std::endl;
//    }
//    outfile.close();
}
