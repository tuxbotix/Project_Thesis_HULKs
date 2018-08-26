# Introduction
Code of Project thesis done with HULKs. This depends on the current HULKs framework. Didn't test with 2017 code release.

# What does it do?
1. Generate poses for the Nao with some limits.
2. Find out how sensitive is the observation model [Currently camera only, x, y and z-rotation in camera sensor plane.] for a given pose, per given joint (slight angle differene is induced to see the observation).
3. Run the sensitivity measurements for every valid pose, and store the values.
4. Now the measurements can be clustered [normalized direction] and choose highest magnitudes as best poses. Also this approach helps to identify possible ambiguities (ie: two or more joints exhibiting same observation direction).
5. Next step is not really done on this repo, but on Nao and MATE, to command each pose, get measurements and run non-linear optimization.

# Pose Generation

Can be done in two ways, forward or inverse kinematics.
Forward is nicer as it give uniform value steps but the amount of poses generated is very high.
Inverse is good in the sense the poses give a direct idea how the legs and head will be positioned, also it's easier to specify the work envelope (ie: support feet, must be standing, etc).
Therefore inverse kinematic approach is used with a port of the pose generation module done in January. Even then, several million poses are there, so naturally poses must be filtered based on sensitivity and orthogonality (un-ambiguousness :P )

# Observation Model

Here a given joint set is taken, and camera2ground matrix is built. then a grid of points ("vector<Vector2f> grid" in source) in robot coordinates is transformed to camera sensor plane.
This will be called "vector<Vector2f> baselinePoints" in the source code.

Next, for each joint:
* A small variation is applied, camera2ground matrix is updated and "observedPoints" are calculated by transforming the "grid".
Now there are two grids in camera plane.

* In theory, it is possible to observe almost complete 6 DOF movement of the camera with the camera sensor, but it's not that notceable for some movements.
Therefore it is simplified to assume only x, y translations and z-rotation is observed. It should be noted that these 3 dimensions are orthogonal.

* Based on this assumption, pose2D (x,y translations and z rotation) is calculated using baselineGrid and observedGrid.
* The output is a Vector3f with x, y and z rotation values. They are not scaled.

At the end of this loop, a PoseSensitivity object is created which holds the observed sensitivity per each joint at the given pose.

Now this can be used for the clustering and further analysis.

# Clustering

## Why?
Clustering can be used in two ways,
* As raw sensitivity cluster (an octree can be used for example). This can give an idea of strongest sensitivities but the direction of observation isn't directly observable.
* Clustering by normalized directions. I believe this is more useful as it gives the general direction of observation for each joint at a given pose.
  * Also I can compare the clusters of each joint with other joints per each pose. If two (or more) joints have same observation direction for a given pose, then that creates an ambiguity. Therefore, that pose isn't suitable for the observation, thus the next strongest pose for each of the joint can be selected.
  * At the worst case, there won't be any poses that doesn't exhibit the ambiguity for the sensor.
  * Still this isn't that bad if: j1, j2 have common pose while j2 and j3 or j2 alone have another sensitive pose. Then both poses can be used together as the optimizer will filter out which is which [ basics of simultanous equations :P ]
  * It's not the number of equations that matter, it's the orthogonality of the configurations that matter.

This section isn't fully clear yet and needs further work.

# End game.

Assuming a good set of poses is extracted, now the robot can take each pose, get the measurements with marker pattern and use a non-linear optimizer to get the joint angle errors.

If fails, needs to investigate why!
