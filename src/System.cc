// Copyright 2008 Isis Innovation Limited
// 系统类 主函数 
#include "System.h"

#include "OpenGL.h"
#include "ATANCamera.h"
#include "MapMaker.h"
#include "Tracker.h"
#include "ARDriver.h"
#include "MapViewer.h"

using namespace CVD;// libCVD      图像处理、视频捕捉、计算机视觉库
using namespace std;// standard
using namespace GVars3;// 系统配置库，属于libCVD的子项目，功能是读取配置文件，获取命令行数
// 系统类 System::System() 类构造函数
System::System()
    : mpVideoSource(new VideoSourceV4L()) 
    , mGLWindow(mpVideoSource->Size(), "PTAM")
{
// 1. 注册一系列命令、添加相对应的功能按钮。
    GUI.RegisterCommand("exit", GUICommandCallBack, this);// 退出
    GUI.RegisterCommand("quit", GUICommandCallBack, this);// 停止
// 2. 检查相机参数是否已经传入，否则退出，去进行相机标定 
    // First, check if the camera is calibrated.
    // If not, we need to run the calibration widget.
    Vector<NUMTRACKERCAMPARAMETERS> vTest;
    vTest = GV3::get<Vector<NUMTRACKERCAMPARAMETERS> >("Camera.Parameters", ATANCamera::mvDefaultParams, HIDDEN);
    if(vTest == ATANCamera::mvDefaultParams)
    {
        cout << endl;
        cout << "! Camera.Parameters is not set, need to run the CameraCalibrator tool" << endl;
        cout << "  and/or put the Camera.Parameters= line into the appropriate .cfg file." << endl;
        exit(1);
    }
// 3. 创建摄像机ATANCamera对象 
    // 加载camera.cfg配置文件中的摄像机内参fx fy cy cy,相机畸变参数
    // 设置黑白图像及彩色图像尺寸、计算从图像坐标系到z=1屏幕投影(归一化平面)及相应像素大小、
    // 找出在z=1平面内的边界四边形并计算线性投影所需参数。
    mpCamera = new ATANCamera("Camera");
// 4. 创建地图Map 地图创建管理器MapMaker 跟踪器Tracker 增强现实AR驱动ARDriver 地图显示MapViewer
    mpMap = new Map;
    mpMapMaker = new MapMaker(*mpMap, *mpCamera);
    mpTracker = new Tracker(mpVideoSource->Size(), *mpCamera, *mpMap, *mpMapMaker);
    mpARDriver = new ARDriver(*mpCamera, mpVideoSource->Size(), mGLWindow);
    mpMapViewer = new MapViewer(*mpMap, mGLWindow);
// 5. 初始化游戏菜单及相应功能按钮。
    GUI.ParseLine("GLWindow.AddMenu Menu Menu");
    GUI.ParseLine("Menu.ShowMenu Root");
    GUI.ParseLine("Menu.AddMenuButton Root Reset Reset Root");
    GUI.ParseLine("Menu.AddMenuButton Root Spacebar PokeTracker Root");
    GUI.ParseLine("DrawAR=0");
    GUI.ParseLine("DrawMap=0");
    GUI.ParseLine("Menu.AddMenuToggle Root \"View Map\" DrawMap Root");
    GUI.ParseLine("Menu.AddMenuToggle Root \"Draw AR\" DrawAR Root");

    mbDone = false;//初始化时mbDone = false; 
}

// 系统运行函数
void System::Run()
{
    while(!mbDone)
    {
        // 1. 创建图像处理对象
        CVD::Image<CVD::Rgb<CVD::byte> > imFrameRGB(mpVideoSource->Size());// 彩色图像用于最终的显示
        CVD::Image<CVD::byte> imFrameBW(mpVideoSource->Size());//黑白(灰度)图像用于处理追踪相关等功能
        // 2. 采集上述两种图像
        mpVideoSource->GetAndFillFrameBWandRGB(imFrameBW, imFrameRGB);
        // 3. 系统跟踪和建图， 更新系统帧
        UpdateFrame(imFrameBW, imFrameRGB);
    }
}

/**
 * @brief 更新系统
 * @param imBW   灰度图 gray image
 * @param imRGB  彩色图 color image
 */
void System::UpdateFrame(Image<byte> imBW, Image<Rgb<byte> > imRGB)
{
// 1. 第一帧的处理 
    static bool bFirstFrame = true;// 第一帧标志
    if(bFirstFrame)
    {
        mpARDriver->Init();// 初始化ARDriver::mpARDriver->Init();这里主要用于生成纹理标识及FrameBuffer，与OpenGL相关
        bFirstFrame = false;// 处理之后，标志复位
    }
// 2. 设置窗口相关属性
    mGLWindow.SetupViewport();
    mGLWindow.SetupVideoOrtho();
    mGLWindow.SetupVideoRasterPosAndZoom();

    if(!mpMap->IsGood())// 地图不好
        mpARDriver->Reset();// 重置，重新开始 
// 3. 读取 显示配置参数
    static gvar3<int> gvnDrawMap("DrawMap", 0, HIDDEN|SILENT);
    static gvar3<int> gvnDrawAR("DrawAR", 0, HIDDEN|SILENT);
// 4. DrawMap及DrawAR状态变量的判断
    bool bDrawMap = mpMap->IsGood() && *gvnDrawMap;
    bool bDrawAR = mpMap->IsGood() && *gvnDrawAR;
// 5. 开始追踪黑白图像(相机位姿跟踪)
    // 多层级金字塔图像(多金字塔尺度) FAST角点检测匹配跟踪
    // 每一个层级的阈值有所不同。最后生成按列角点查询表，便于以后近邻角点的查询任务.
    mpTracker->TrackFrame(imBW, !bDrawAR && !bDrawMap);
    
// 6. 可视化显示
    if(bDrawMap)
        mpMapViewer->DrawMap(mpTracker->GetCurrentPose());// 显示地图点云
    else if(bDrawAR)
        mpARDriver->Render(imRGB, mpTracker->GetCurrentPose());//显示虚拟物体
// 7.可视化文字菜单显示
    string sCaption;
    if(bDrawMap)
        sCaption = mpMapViewer->GetMessageForUser();
    else
        sCaption = mpTracker->GetMessageForUser();
    mGLWindow.DrawCaption(sCaption);
    mGLWindow.DrawMenus();
    mGLWindow.swap_buffers();
    mGLWindow.HandlePendingEvents();
}
// GUI菜单 命令 回调函数检测停止/退出按键
void System::GUICommandCallBack(void *ptr, string sCommand, string sParams)
{
    if(sCommand=="quit" || sCommand == "exit")// 停止/退出标志
        static_cast<System*>(ptr)->mbDone = true;
}
// 系统析构函数
System::~System()
{
    if(mpVideoSource!=NULL)
    {
        delete mpVideoSource;// delete 
        mpVideoSource = NULL;
    }
}
