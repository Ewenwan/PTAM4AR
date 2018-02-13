#include "Common.h"
#include "VideoSource.h"

#include <gtest/gtest.h>

TEST(ImageDataSet,ReadImagesAssociationFile)
{
    VideoSource *videoSource = new ImageDataSet("/home/gordon/projects/PTAM4AR/data/rgbd_dataset_freiburg1_xyz",
                                                "/home/gordon/projects/PTAM4AR/data/rgbd_dataset_freiburg1_xyz/associate.txt");
    cv::Mat imgRGB, imgBW;
    int ret = videoSource->GetFrameRGBBW(imgRGB, imgBW);
    ASSERT_TRUE(ret==GS::RET_SUCESS);
    if(videoSource!=NULL)
    {
        delete videoSource;
        videoSource = NULL;
    }
    ASSERT_TRUE(!imgRGB.empty());
}
