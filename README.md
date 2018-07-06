# Parallel Tracking And Mapping (PTAM) 特征点法 
![](http://image.mamicode.com/info/201802/20180211193608683439.png)
      
      PTAM[1]是视觉SLAM领域里程碑式的项目。
      在此之前，MonoSLAM[2]为代表的基于卡尔曼滤波的算法架构是主流，
      它用单个线程逐帧更新相机位置姿态和地图。
      
      
      2007年，Klein等人提出了PTAM（Parallel Tracking and Mapping），
      这也是视觉SLAM发展过程中的重要事件。
      
      PTAM的重要意义在于以下两点：
      
      1、PTAM提出并实现了 跟踪 与 建图 过程的并行化。
         我们现在已然清楚，跟踪部分需要实时响应图像数据，而对地图的优化则没必要实时地计算。
         后端优化可以在后台慢慢进行，然后在必要的时候进行线程同步即可。
         这是视觉SLAM中首次区分出前后端的概念，引领了后来许多视觉SLAM系统的设计（我们现在看到的SLAM多半都分前后端）。
         
      2、PTAM是第一个使用 非线性优化，而不是使用传统的 滤波器 作为后端的方案。
         它引入了关键帧机制：
               我们不必精细地处理每一幅图像，而是把几个关键图像串起来，然后优化其轨迹和地图。
               早期的SLAM大多数使用EKF滤波器或其变种，以及粒子滤波器等；
               在PTAM之后，视觉SLAM研究逐渐转向了以非线性优化为主导的后端。
               由于之前人们未认识到后端优化的稀疏性，
               所以觉得优化后端无法实时处理那样大规模的数据，而PTAM则是一个显著的反例。
      
      根据PTAM估计的相机位姿，我们可以在一个虚拟的平面上放置虚拟物体，看起来就像在真实的场景中一样。
      
      
      PTAM是增强现实相关的一个库，与SLAM框架类似，兼容ROS。
      ORB-SLAM就是继承并改进PTAM的
      
      PTAM，全称Parallel Tracking And Mapping，
      是最早提出将Track和Map分开作为两个线程的一种SLAM算法，
      是一种基于关键帧的单目视觉SLAM算法。
      
      PTAM（Parallel Tracking and Mapping）架构更多的是系统上的设计，
      姿态跟踪（Tracking）和建立地图（Mapping）两个线程是并行的，这实质上是一种针对SLAM的多线程设计。
      PTAM在当前SLAM领域看来是小儿科，但在当时是一个创举，
      第一次让大家觉得对地图的优化可以整合到实时计算中，并且整个系统可以跑起来。
      
      具体而言: 
         1. 姿态跟踪线程 不修改地图，只是利用已知地图来快速跟踪；
         2. 而在建立 地图线程 专注于地图的建立、维护和更新。
         
      即使建立地图线程耗时稍长，姿态跟踪线程仍然有地图可以跟踪（如果设备还在已建成的地图范围内）.
      这是两个事情并行来做的一个好处，但很现实的问题是如果地图建立或优化过慢，
      跟踪线程很容易会因为没有最新的地图或者没有优化过的地图而跟丢。
      
      另外比较实际的工程问题是地图线程的最新地图数据应该lock 
      还是 copy data between threads
      以及threading的实现质量。
      
      PTAM主要分为这几部分：
            1) Track线程
                1. 金字塔分层，FAST特征提取
                   (对图片构造金字塔的目的有两个：1）加快匹配；2）提高地图点相对于相机远近变化时的鲁棒性)
                   (FAST是常用的特征点，优点是快，缺点是不鲁棒.通常先提取出大量匹配点，后使用SSD快匹配，剔除误匹配)
                2. 地图初始化
                3. 跟踪定位
               　　（极线几何与极线搜索，
               　　  RANSAC（随机采样一致）及N点算法（主要围绕5点算法））
                4. 选取添加关键帧到缓存队列
                5. 重定位(每帧高斯模糊小图SSD相似度匹配)
            2) Map线程
                5.5 先五点法加RANSAC求出初值
                6. 局部BundleAdjustment
                7. 全局BundleAdjustment
                8. 从缓存队列取出关键帧到地图
                9. 极线搜索加点到地图
                
      另一方面，按照一般的视觉SLAM框架，PTAM也可分为
            1) 传感器数据获取（摄像头输入图像数据）
            2) 前端视觉里程计（跟踪定位、重定位）
            3) 后端优化（Bundle Adjustment）
            4) 建图（极线搜索加点）
            5) 没有回环检测

# PTAM4AR

[![Join the chat at https://gitter.im/PTAM4AR/Lobby](https://badges.gitter.im/PTAM4AR/Lobby.svg)](https://gitter.im/PTAM4AR/Lobby?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge) [![Documentation](https://codedocs.xyz/GaoHongchen/PTAM4AR.svg)](https://codedocs.xyz/GaoHongchen/PTAM4AR/)  
PTAM source code for AR based on the one from Isis.

# Introduction
PTAM (Parallel Tracking and Mapping) is a camera tracking system for augmented reality.

# Building

## Build Status on CI
* Linux (Ubuntu-14.04 64 bits, GCC-5) : [![Build Status](https://travis-ci.org/GaoHongchen/PTAM4AR.svg?branch=master)](https://travis-ci.org/GaoHongchen/PTAM4AR) [![Coverage Status](https://coveralls.io/repos/github/GaoHongchen/PTAM4AR/badge.svg?branch=master)](https://coveralls.io/github/GaoHongchen/PTAM4AR?branch=master)

## Build This Project locally

**1) Install dependencies**
* Execute the script to install dependencies or 3rdParties on Linux: [3rdParty_install.sh](./scripts/3rdParty_install.sh)   

**2) Build Project**
* Execute the script: [build_project.sh](./scripts/build_project.sh)

## Relate Compilation for PTAM
* [PTAM Compilation on Linux](http://hustcalm.me/blog/2013/09/27/ptam-compilation-on-linux-howto/)
* [Build PTAM on Ubuntu 11.10](http://irawiki.disco.unimib.it/irawiki/index.php/PTAM)
* [Installation of PTAM in ROS](https://sites.google.com/site/zhilongliuwebsite/research/computer-vision-embedded-systems/ptam)

# Tools

## Static Analysis of C/C++
* cppcheck: [Add cppcheck and clang-format for a cmake project](https://arcanis.me/en/2015/10/17/cppcheck-and-clang-format)

## Test Code Coverage
* coveralls-cmake

## Generate API documentation
* Doxygen

# Relate Source Code

## Linux
* [PTAM-GPL (GitHub)](https://github.com/Oxford-PTAM/PTAM-GPL)
* [PTAM-linux-cv2.3 (GitHub)](https://github.com/nttputus/PTAM-linux-cv2.3)

## Android
* [APTAM-GPL](https://github.com/ICGJKU/APTAM-GPL)
* [android-ptam](https://github.com/damienfir/android-ptam)

## Windows
* [PTAM-Windows](https://github.com/LucRyan/PTAM-Windows)

# PTAM Tutorials
* [PTAM Official Site](http://www.robots.ox.ac.uk/~gk/PTAM/)
* [PTAM news](https://ewokrampage.wordpress.com/)
* [PTAM-Monocular SLAM](http://www.doc.ic.ac.uk/~gj414/monocular_slam/ptam.html)
* [implementing-ptam-stereo-tracking-and-pose-estimation-for-ar-with-opencv-w-code](http://www.morethantechnical.com/2010/03/06/implementing-ptam-stereo-tracking-and-pose-estimation-for-ar-with-opencv-w-code/)
* [PTAM on ROS](http://wiki.ros.org/ptam)

# SLAM Resources
* [SLAM学习与开发经验分享(Lee-SLAM-source)](https://github.com/AlbertSlam/Lee-SLAM-source)
* [高翔 slambook code](https://github.com/gaoxiang12/slambook)
