# Literature Review

## Nao Hardware

## HULKs framework

## Camera calibration

Camera calibration is a well researched topic as it is a primary dependancy for obtaining good results with camera systems. In this thesis, these standard methods & tools for intrinsic calibration is employed, while discussion about extrinsic calibration approaches will be given higher regard.

### Method of HULKs

The calibration framework is based on the debug tool used by HULKs and the method of operation and usage is explained in detail in the team research report. In summary, the calibration is achieved by capturing images and kinematic chain with time synchronization to solve the following system of equations .with a non linear solver (LM).

### Others

The team B-Human... , etc do different approaches for extrinsics....

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
There are several experiments done in context of the Nao Robot for indirect joint calibration. Some involves fixing a calibration pattern in the form of a sticker [].. The experiment by the team B-Human involves "sandles" worn by the Nao, but untimately their results were less than satisfactory. A few possible causes include the robot taking measurements while lying on its back which is not a typical pose during robocup games. 

In addition, there isn't much previous research regarding the observability of joint errors with on board sensors alone. Therefore one of the key components of this thesis would be devising an observation model for joint errors and deriving most suitable poses.

B-Human paper - joint calib
The y don’t specifically investigate observability of error of a given joint or derive opti-
mal set of poses for the task.

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

