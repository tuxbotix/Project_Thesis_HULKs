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