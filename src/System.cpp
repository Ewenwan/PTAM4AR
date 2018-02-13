#include "System.h"

#include <unistd.h>
#include <iostream>

#include "Common.h"
#include "JsonConfig.h"
#include "GLWindowPangolin.h"
#include "VideoSource.h"
#include "ATANCamera.h"
#include "MapMaker.h"
#include "Tracker.h"

System::System()
{
    mpJsonConfig = new JsonConfig("../Config.json");
    mpPangolinWindow = new GLWindowPangolin("PTAM-GS",GS::Size(640,480));
    mpVideoSource = new ImageDataSet("/home/gordon/projects/PTAM4AR/data/rgbd_dataset_freiburg1_xyz",
                                       "/home/gordon/projects/PTAM4AR/data/rgbd_dataset_freiburg1_xyz/associate.txt");
    mpCamera = new ATANCamera();
    mpMapMaker = new MapMaker(*mpCamera);
    mpTracker = new Tracker(mpJsonConfig, mpPangolinWindow, *mpMapMaker);

    mbDone = false;
}

void System::Run()
{
    if(GS::RET_FAILED == mpJsonConfig->Init())
        return;

    cv::Mat imgRGB,imgBW;
    while(!mbDone)
    {
        if(GS::RET_FAILED == mpVideoSource->GetFrameRGBBW(imgRGB,imgBW))
            break;

        mpPangolinWindow->SetupViewport();
        mpPangolinWindow->SetupVideoOrtho();
        mpPangolinWindow->SetupVideoRasterPosAndZoom();

        mpPangolinWindow->Clear();
        mpPangolinWindow->DrawTexture2DGray(imgBW);

        Update(imgBW,imgRGB);

        mpPangolinWindow->RenderTextureToViewport();
        mpPangolinWindow->EndFrame();

        usleep(40000);
    }
}

void System::Update(const cv::Mat &imgBW, const cv::Mat &imgRGB)
{
    mpTracker->TrackFrame(imgBW, true);
}

System::~System()
{
    DELETE_NEW_OBJ(mpTracker)
    DELETE_NEW_OBJ(mpMapMaker)
    DELETE_NEW_OBJ(mpCamera)
    DELETE_NEW_OBJ(mpVideoSource)
    DELETE_NEW_OBJ(mpPangolinWindow)
    DELETE_NEW_OBJ(mpJsonConfig)
}
