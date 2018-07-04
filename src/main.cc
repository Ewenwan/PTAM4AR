// Copyright 2008 Isis Innovation Limited
// This is the main extry point for PTAM
// 系统入口main函数
#include <stdlib.h>
#include <iostream>

#include "System.h"

using namespace std;
using namespace GVars3;

int main()
{
    cout << "  Welcome to PTAM " << endl;
    cout << "  --------------- " << endl;
    cout << "  Parallel tracking and mapping for Small AR workspaces" << endl;
    cout << "  Copyright (C) Isis Innovation Limited 2008 " << endl;
    cout << endl;

    cout << "  Parsing settings.cfg ...." << endl;
    GUI.LoadFile("../config/settings.cfg");// 载入相机参数等配置文件
    GUI.StartParserThread(); // 用户界面线程启动 Start parsing of the console input
    atexit(GUI.StopParserThread);

    try
    {
        System s;// 创建系统对象
        s.Run();// 运行
    }
    catch(CVD::Exceptions::All e)// 异常状态捕获
    {
        cout << endl;
        cout << "!! Failed to run system; got exception. " << endl;
        cout << "   Exception was: " << endl;
        cout << e.what << endl;
    }
}
