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



