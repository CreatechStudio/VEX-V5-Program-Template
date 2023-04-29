# VEX V5 Program Template

## 简介

​	这是一个基于VEX V5竞赛模板的基本程序，它由C++编写，包含了

- 基本的底盘控制
- PID控制
- PID调参
- 基于电机编码器与陀螺仪的GPS定位
- 自动框架

## 文件结构

- 根目录

  - data

    <!--MATLAB PID adjustment-->

    - pid_base.m

  - include

    <!--header file-->

    - GPS.h
    - adjusment.h
    - base.h
    - controller.h
    - my-timer.h
    - robot-config.h
    - vex.h
    - PID.h
    - autonomous.h
    - basic-functions.h
    - math-tools.h
    - parameters.h
    - sensors.h

  - src

    <!--source file-->

    - GPS.cpp
    - adjustment.cpp
    - base.cpp
    - main.cpp
    - my-timer.cpp
    - sensors.cpp
    - PID.cpp
    - autonomous.cpp
    - basic-functions.cpp  math-tools.cpp
    - robot-config.cpp

  - vex

    <!--vex basic file-->

    - mkenv.mk
    - mkrules.mk