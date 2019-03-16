# Evaluation 1

This evaluation is meant to validate the main systems by simulating the robot with its kinematic chains and a grid of relatively dense (compared to a dense point cloud) 3D points acting as calibration features.

This is conducted against following sets of calibration poses obtained against simulated environmental conditions. The following matrix depicts the total combinations. This is intended to give detailed overview of factors which affect calibration and give insight what to be done for next steps.

|             | Env. Config 1 | Env. Config 2 | Env. Config 3 | Env. Config 4 |
| ----------- | ------------- | ------------- | ------------- | ------------- |
| Input Set 1 | x             | x             | x             | x             |
| Input Set 2 | x             | x             | x             | x             |
| Input Set 3 | x             | x             | x             | x             |
|             |               |               |               |               |

### Input Data

The inputs are poses to calibrate and test poses.

1. Optimal chain of poses (Obtained by previously discussed process)
2. Poses from cost function only (Prior to building the chain)
3. Random set of poses

### Parameters:

* 5 cm x 5 cm (0.05 m x 0.05 m) grid on the ground plane
* max view dist = 4.0 m
* Good calibration:
  * 0.2° > absolute joint error after calibration (must pass for all joints). = No local minima and 'match' with ground truth. Can only be evaluated with ground truth.
  * 2% of min(image Size) < average absolute reprojection error: Used to identify solver convergence. [The test data set might help to correlate between local minima and global minima situations]
  * Solver exit without error state.
* induced error:
  * +-6.5 degree range
  * Uniform distribution
* population size
  * 1000
  * 10000

### Environmental Configurations (simulated):

This was conducted in several steps, to simulate ideal conditions and to observe effects of effects such as noise.

* No noise
* Noise into pixel positions of calibration points viewed from the camera (simulated).
  * 0 centered Gaussian distribution, Standard deviation 1.0
* Noise into joint sensor values. (simulated)
  * 0 centered Gaussian distribution, Standard deviation 0.01
* Both combined

### Tables

Success rate for each situation, which joints survived better

### Plots

1. Induced and calibrated error distributions (overall and per joint)
2. Reprojection error (average, std. dev)
   1. Calibration poses
   2. Verification poses
3. Head Pose [3D] (avg., std. dev) [**Optional**]
   1. Calibration Poses
   2. Verification poses
4. Number of poses vs success rate [If possible, do joint-by-joint basis]
   1. Calibration Poses
   2. Verification poses
5. Which error configs. cause most failures? [**optional**]