# R3LIVE
## A Robust, Real-time, RGB-colored, LiDAR-Inertial-Visual tightly-coupled state Estimation and mapping package

**[Updated][Dec 29, 2021] Release of datasets**: Our datasets for evaluation can now be accessed from [Google drive](https://drive.google.com/drive/folders/15i-TRa0EA8BCbNdARVqPMDsU9JOlagVF?usp=sharing) or [Baidu-NetDisk [百度网盘]](https://pan.baidu.com/s/1zmVxkcwOSul8oTBwaHfuFg) (code提取码: wwxw). We have released totally **9** rosbag files for evaluating r3live, with the introduction of these datasets can be found on this [page](https://github.com/ziv-lin/r3live_dataset). And, as claimed before, our source code will be released before Dec 31, 2021.

**Our preprint paper**: Our preprint paper are available at [here](https://github.com/hku-mars/r3live/blob/master/papers/R3LIVE:%20A%20Robust%2C%20Real-time%2C%20RGB-colored%2C%20LiDAR-Inertial-Visual%20tightly-coupled%20stateEstimation%20and%20mapping%20package.pdf)

**Data of release**: We have just received the reviewer comments in the first round of paper reviews and we decide to release our codes before **Dec 31, 2021**.

**Our related video**: our related video is now available on YouTube (click below images to open, or watch it on Bilibili<sup>[1](https://www.bilibili.com/video/BV1d341117d6?share_source=copy_web), [2](https://www.bilibili.com/video/BV1e3411q7Di?share_source=copy_web)</sup>):
<div align="center">
<a href="https://youtu.be/j5fT8NE5fdg" target="_blank"><img src="pics/R3LIVE_paper.png" alt="video" width="45%" /></a>
<a href="https://youtu.be/4rjrrLgL3nk" target="_blank"><img src="pics/R3LIVE_demos.png" alt="video" width="45%" /></a>
</div>

## Introduction
&emsp;**R3LIVE** is a novel LiDAR-Inertial-Visual sensor fusion framework, which takes advantage of measurement of LiDAR, inertial, and visual sensors to achieve robust and accurate state estimation. R3LIVE is contained of two subsystems, the LiDAR-inertial odometry (LIO) and visual-inertial odometry (VIO). The LIO subsystem ([FAST-LIO](https://github.com/hku-mars/FAST_LIO)) takes advantage of the measurement from LiDAR and inertial sensors and builds the geometry structure of (i.e. the position of 3D points) global maps. The VIO subsystem utilizes the data of visual-inertial sensors and renders the map's texture (i.e. the color of 3D points). 

<div align="center">
    <div align="center">
        <img src="pics/overview.png" width = 80% >
    </div>
    <font color=#a0a0a0 size=2>The overview of this package</font>
    <div align="center">
        <img src="pics/cover.jpg" width = 80% >
    </div>
    <font color=#a0a0a0 size=2>(a): R3LIVE is able to reconstruct a dense, 3D, RGB-colored point cloud of the traveled environment in real-time. The white path is our traveling trajectory for collecting the data. (b): The mesh reconstructed with our offline utilities. (c): The mesh after textured with the vertex colors, which is rendered by our VIO subsystem</font>
</div>

&emsp;**R3LIVE** is developed based on our previous work [R2LIVE](https://github.com/hku-mars/r2live), with careful architecture design and implementation, is a versatile and well-engineered system toward various possible applications, which can not only serve as a SLAM system for real-time robotic applications, but can also reconstruct the dense, precise, RGB-colored 3D maps for applications like surveying and mapping. Moreover, to make R3LIVE more extensible, we develop a series of offline utilities for reconstructing and texturing meshes, which further minimizes the gap between R3LIVE and various of 3D applications such as simulators, video games and etc.

<div align="center">
    <div align="center">
        <img src="pics/r3live_airsim.png" width = 80% >
    </div>
    <font color=#a0a0a0 size=2>We use the maps reconstructed by R3LIVE to build the car (in (a)) and drone (in (b)) simulator with AirSim(https://microsoft.github.io/AirSim). The images in green and blue frameboxes are of the depth, RGB image query from the AirSim's API, respectively.</font>
    <div align="center">
        <img src="pics/r3live_ue.png" width = 80% >
    </div>
    <font color=#a0a0a0 size=2>We use the maps built by R3LIVE to develop the video games for mobile platform (see (a)) and desktop PC (see (b)). In (a), the player is controling the actor to explore the campus of HKU. In (b), the player is fighting against the dragon with shoting the rubber balls in the campus of HKUST.</font>
</div>

## Relative works
1. [FAST-LIO](https://github.com/hku-mars/FAST_LIO): A computationally efficient and robust LiDAR-inertial odometry package.
2. [R2LIVE](https://github.com/hku-mars/r2live): A robust, real-time tightly-coupled multi-sensor fusion package.
3. [ikd-Tree](https://github.com/hku-mars/ikd-Tree): A state-of-art dynamic KD-Tree for 3D kNN search. 
4. [LOAM-Livox](https://github.com/hku-mars/loam_livox): A robust LiDAR Odometry and Mapping (LOAM) package for Livox-LiDAR