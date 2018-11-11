# Introduction

Robots are increasingly becoming human-friendly and humanoid robots are already considered for roles from learning tools for children to caring the elderly to play soccer.

The RoboCup was initiated in 1997 as an annual competition in robotics with main focus on playing Soccer.

This thesis is written regarding calibrating Nao, the humanoid robot by Softbank robotics. While the research presented here can be applied to other humanoid and serial robot platforms, the foundations lies in the context of RoboCup, where the team HULKs from Hamburg University of technology (TUHH) participate in the Standard Platform League. In addition to the technical challenges in calibration, the international and competitive nature of robocup also make it nessesary to consider factors like time to calibrate, logistics. 

This chapter will introduce more about the robocup, the Nao robot and the motivations for this thesis.

## Robocup

With the vision of robots playing against FIFA champions in the year 2050, the Robocup was inaugurated 21 years ago (1997) in Japan. Currently the tournament hosts multiple leagues of humanoid robot soccer, but also hosts competitions in other areas such as logistics and rescue. The specialty of Robocup is that not only it is a competition, but it also act as a platform to share research, ideas and implementations (source code, etc) in robotics.

### The standard Platform League

The RoboCup Standard Platform League comprise of teams playing against each other using the same type of Hardware. Initially the Sony Aibo robot "dog" was used as the platform and since 2008, the Nao Robot - a Humanoid robot from Softbank Robotics is used. 

## The Nao Robotics system

TODO

## Motivation

Calibrating the two cameras of the nao has been a prerequistic for obtaining good coordination of the robots and several methods has been derived at HULKs for the purpose of Camera Calibration (Intrinsic and Extrinsic).

In the recent years, with the aging Nao V5 robots it was observed that the joints of the Nao have a significant play and backlash. In certain occasions, the origin of the joint position sensors were also found to be shifted as much as 20 degrees. For a joint such as the neck, this means the error of perception is easily in the magnitude of several errors.

Therefore it has come to attention the need for automatic joint error detection and calibration as this also affects the camera extrinsic calibration process.

In the past several teams has attemped this with less than satisfactory results and as far as it is known, the only working process seems to be manual measurement and adjustment.

## Goals

The aim of this thesis is to develop a fully or semi-automatic joint and camera calibration. This includes theoritical observer modelling, optimal calibration pose generation, calculation of potential success of calibrating a given joint and the implementation of the nessesary tools, modules into the HULKs Nao framework and the debug tooling (which will be extending the current calibration tools).

Investigate current approaches and determine issues with them

* Derive a process to determine suitability of a sensors, poses, input data, etc. ->
  also observer model
* Investigate the suitability of onboard sensors based on the above mentioned pro-
  cess. (at least some joints must be calibrate-able)
* Determine the effect of backlash and offsets, level of observation and other fac-
  tors.

However, it should be noted that the primary goal of this thesis is to provide a foundation on observer models, deriving them and a process to determine suitability, Not to calibrate robots to perfection as end result.**

## Scope

Due to the vast number of possible avenues and requirements, this thesis is constrained in the following scope.

* Only on-board sensors are evaluated in this thesis
* Primary testing will be with different levels simulations.
* Once simulation based testing proves feasibility, use real robots.

# Literature Review

## Nao Hardware



## HULKs NAO framework

This is the software framework developed by HULKs to control the NAO for playing in RoboCup SPL written in C++. The framework also supports debugging and configuration via the Debug tool "MATE" - also developed by HULKs. Further information is present in the Team Research Report.

## Camera calibration

Camera calibration is a well researched topic as it is a primary dependancy for obtaining good results with camera systems. In this thesis, these standard methods & tools for intrinsic calibration is employed, while discussion about extrinsic calibration approaches will be given higher regard.

### Method of HULKs

The calibration process is based on the debug tool used by HULKs (MATE) as well as the HULKs NAO framework. The method of operation and usage is explained in detail in the team research report. In summary, the calibration is achieved by capturing images and kinematic chain with time synchronization to solve the following system of equations .with a non linear solver (LM).

### Others

The team B-Human employs a method combined with joint calibration. \todo{Cite paper BHUMAN} HTWK-Lepzig employs a method based on the center circle, their robots gather enough feature points in several poses to solve the non linear equation. Berlin United also employs a similar approach but they do this with the robots in a sit down (unstiffed) pose and they can use all the lines of the entire field as features.

### Calibration features

#### Patterns:

Charuco - a hybridization of the classic chessboard and AR markers to achieve fast detection and obtaining 3D points for partially visible patterns. ( Traditional implementations using chessboard relies on having complete visibility of the chessboard).

## Joint calibration

### Joint error types

### Direct measurement

### Indirect measurement

In practical scenarios, indirect measurement based methods are more useful as they
involve in less specialized sensors or eliminate need to measure each sensor individually.

### Previous work?

There are several experiments done in context of the Nao Robot for indirect joint calibration. Some involves fixing a calibration pattern in the form of a sticker [].. One of the experiment by the team B-Human involves "sandles" worn by the Nao, but untimately their results were less than satisfactory. A few possible causes include the robot taking measurements while lying on its back which is not a typical pose during robocup games. \todo{add citation, add table with results}

However, they currently have a working joint calibration method based on manual observations. First, the robot is put to a specific pose (ie: standing), lifted to observe foot offsets and adjustments are done so that both feet are at same level and orientation. Next, the robot is placed on the ground and the distance to hip or another known joint origin is measured. This allows to measure the length of the kinematic chain of each leg. Any deviations are compensated by means of inverse kinematic values and finally the robot's leaning \todo{clarify} towards ground is also adjusted. While this appears to be a practical method, the disadvantage of manual measurements is time consuming and can be erroneous.

There isn't significant amount of previous research regarding the observability of joint errors with on board sensors alone. Therefore one of the key components of this thesis would be devising an observation model for joint errors and deriving most suitable poses. This should assist to explain the quality of calibration at each joint.

## Observation Models

### Ambiguities of observations

...

Cosing similarity,,

## Computing a pose for a robot

### Forward Kinematics

### Inverse Kinematics

## Cost functions, Optimization, clustering

### Cost functions and role in this regard

### Optimization approaches and clustering

### Curse of dimensionality

# Methodology

## Workflow

The below workflow was followed during this thesis. Each component is explained in detail in the upcoming sections.



Here a given joint set is taken, and camera2ground matrix is built. then a grid of points ("vector<Vector2f> grid" in source) in robot coordinates is transformed to camera sensor plane.
This will be called "vector<Vector2f> baselinePoints" in the source code.

Next, for each joint:

- A small variation is applied, camera2ground matrix is updated and "observedPoints" are calculated by transforming the "grid".
  Now there are two grids in camera plane.
- In theory, it is possible to observe almost complete 6 DOF movement of the camera with the camera sensor, but it's not that notceable for some movements.
  Therefore it is simplified to assume only x, y translations and z-rotation is observed. It should be noted that these 3 dimensions are orthogonal.
- Based on this assumption, pose2D (x,y translations and z rotation) is calculated using baselineGrid and observedGrid.
- The output is a Vector3f with x, y and z rotation values. They are not scaled.

At the end of this loop, a PoseSensitivity object is created which holds the observed sensitivity per each joint at the given pose.

Now this can be used for the clustering and further analysis.

