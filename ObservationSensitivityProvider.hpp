#pragma once

#include "Data/CameraMatrix.hpp"

#include "NaoPoseInfo.hpp"
#include "NaoProjectionDataProvider.hpp"

#include "Solvers.hpp"
/**
 * To be precise, this give sensitivity of camera observation :P
 */
class ObservationSensitivity
{
private:
  static constexpr float maxViewDist = 3.0;
  static const size_t maxGridPointsPerSide = 15;

  std::vector<Vector2f> basicGrid;

  Vector2i imSize;
  // float gridSpacing;
  const float deltaThetaCorse;
  const float deltaThetaFine;

  // When divided by this, all outputs must scale between -1 and +1
  const Vector3f dimensionScale;
  // const Vector3f dimensionMax;
  CameraMatrix camMat;

  friend class ObservationSensitivityProvider;
  ObservationSensitivity(const Vector2i &imSize, const Vector2f &fc, const Vector2f &cc, const Vector2f &fov,
                         const float &gridSpacing)
      : imSize(imSize),
        // gridSpacing(gridSpacing),
        deltaThetaCorse(5.0f), deltaThetaFine(1.0f), dimensionScale(imSize.x() / 2, imSize.y() / 2, M_PI_2f64)
  {
    camMat.fc = fc;
    camMat.fc.x() *= imSize.x();
    camMat.fc.y() *= imSize.y();
    camMat.cc = cc;
    camMat.cc.x() *= imSize.x();
    camMat.cc.y() *= imSize.y();
    camMat.fov = fov;

    float x, y;
    const size_t gridSizeHalf = maxGridPointsPerSide / 2;
    for (size_t i = 0; i < maxGridPointsPerSide; i++)
    {
      x = (i * gridSpacing) - gridSizeHalf;
      for (size_t j = 0; j < maxGridPointsPerSide; j++)
      {
        y = (j * gridSpacing) - gridSizeHalf;
        basicGrid.emplace_back(x, y);
      }
    }
  }

public:
  /**
 * Get 3D grid with given grid spacing relative to robot's ground coord.
 * Obviously, camera matrix must be updated before calling this.
 * 1. This has a hard range limit of 2m
 */
  std::vector<Vector2f> getGroundGrid()
  {
    std::vector<Vector2f> output;
    Vector2f robotCoords;
    bool proceed = (camMat.horizonA == 0 && camMat.horizonB == 0.0);
    if (proceed)
    {
      camMat.pixelToRobot(camMat.cc.cast<int>(), robotCoords);
      proceed = (robotCoords.norm() <= maxViewDist);
    }
    if (proceed)
    {
      Vector2f tempCoord;
      // Eigen::Translation<float, 2> trans(tempCoord.x(), tempCoord.y());
      float theta = std::atan2(robotCoords.y(), robotCoords.x());
      Eigen::Rotation2D<float> rot2(theta);
      for (const auto &i : basicGrid)
      {
        tempCoord = robotCoords + (rot2 * i);
        output.push_back(tempCoord);
      }
    }
    return output;
  }

  std::vector<float> robotToPixelMulti(const std::vector<Vector2f> &groundPoints)
  {
    std::vector<float> output;
    Vector2i point;
    for (const auto &i : groundPoints)
    {
      camMat.robotToPixel(i, point);
      output.push_back(point.x());
      output.push_back(point.y());
    }
    return output;
  }

  Vector3f getSensitivityForJointForCamera(const JOINTS::JOINT &joint, const rawPoseT &jointAngles, const KinematicMatrix &sf,
                                           std::vector<Vector2f> grid, const std::vector<float> &baselinePoints,
                                           const Camera &cameraName, bool &observed)
  {
    Vector3f output(0);

    observed = false;
    rawPoseT tempJoints(jointAngles);
    tempJoints[joint] += deltaThetaCorse;

    NaoSensorDataProvider::updatedCameraMatrix(tempJoints, sf, camMat, cameraName);
    std::vector<float> observedPoints = robotToPixelMulti(grid);
    ObservationSolvers::get2dPose(baselinePoints, observedPoints, output);

    return output;
  }

  PoseSensitivity<Vector3f> getSensitivitiesForCamera(const rawPoseT &jointAngles, const SUPPORT_FOOT &sf, const SENSOR_NAME &sensorName)
  {
    if (sensorName != SENSOR_NAME::TOP_CAMERA && sensorName != SENSOR_NAME::BOTTOM_CAMERA)
    {
      throw "Invalid sensor, not camera!";
    }
    Camera cameraName = (sensorName == SENSOR_NAME::TOP_CAMERA) ? Camera::TOP : Camera::BOTTOM;

    PoseSensitivity<Vector3f> output(sensorName);
    KinematicMatrix supFoot = (sf == SUPPORT_FOOT::SF_RIGHT) ? ForwardKinematics::getRFoot(jointAngles) : ForwardKinematics::getLFoot(jointAngles);

    NaoSensorDataProvider::updatedCameraMatrix(jointAngles, supFoot, camMat, cameraName);

    std::vector<Vector2f> grid;
    grid = getGroundGrid();
    std::vector<float> baseLinePoints = robotToPixelMulti(grid);

    bool observed = false;
    if (sf == SUPPORT_FOOT::SF_LEFT || sf == SUPPORT_FOOT::SF_DOUBLE)
    {
      for (const auto &i : Sensor::CAM_OBS_L_SUP_FOOT)
      {
        output.setSensitivity(i, getSensitivityForJointForCamera(i, jointAngles, supFoot, grid, baseLinePoints, cameraName, observed), observed);
      }
    }
    if (sf == SUPPORT_FOOT::SF_RIGHT || sf == SUPPORT_FOOT::SF_DOUBLE)
    {
      for (const auto &i : Sensor::CAM_OBS_R_SUP_FOOT)
      {
        output.setSensitivity(i, getSensitivityForJointForCamera(i, jointAngles, supFoot, grid, baseLinePoints, cameraName, observed), observed);
      }
    }
    return output;
  }
};

class ObservationSensitivityProvider
{
public:
  /**
     * ctor
     * @param threadCount allows automatic initializing of enough camera matrices..
     */

  static std::vector<ObservationSensitivity> getSensitivityProviders(const size_t &threadCount, const Vector2i &imSize,
                                                                     const Vector2f &fc, const Vector2f &cc,
                                                                     const Vector2f &fov, const float &gridSpacing)
  {
    return std::vector<ObservationSensitivity>(threadCount, ObservationSensitivity(imSize, fc, cc, fov, gridSpacing));
  }
};
