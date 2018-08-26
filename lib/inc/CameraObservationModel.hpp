#pragma once

#include <cmath>

#include "Data/CameraMatrix.hpp"

#include "ObservationModel.hpp"
#include "NaoPoseInfo.hpp"
#include "NaoProjectionDataProvider.hpp"

/**
 * To be precise, this give sensitivity of camera observation :P
 */
class CameraObservationModel : public ObservationModel
{
private:
/**
 * To be precise, this give sensitivity of camera observation :P
 */

  static const std::string name;
  static constexpr float maxViewDist= 3.0;

  const size_t maxGridPointsPerSide;

  std::vector<Vector2f> basicGrid;

  Vector2i imSize;
  // float gridSpacing;
  const float deltaThetaCorse;
  const float deltaThetaFine;
  int horizon;
  // When divided by this, all outputs must scale between -1 and +1
  const Vector3f dimensionScale;
  // const Vector3f dimensionMax;
  CameraMatrix camMat;

public:
  ~CameraObservationModel() {}
  // friend class ObservationSensitivity;
  CameraObservationModel(const Vector2i &imSize, const Vector2f &fc, const Vector2f &cc, const Vector2f &fov,
                         const size_t &maxGridPointsPerSide = 15, const float &gridSpacing = 0.05);

  std::string getName()
  {
    return name;
  }

  /**
   * Update the state of the robot.
   */
  void updateState(const rawPoseT &jointAngles, const SUPPORT_FOOT &sf, const SENSOR_NAME &sensorNames);

  /**
   * Get 3D grid with given grid spacing relative to robot's ground coord.
   * Obviously, camera matrix must be updated before calling this.
   * 1. This has a hard range limit of 2m
   */
  // TODO  Make this private.
  std::vector<Vector2f> getGroundGrid();

  /**
   * Robot to pixel, multiple point support
   */
  std::vector<float> robotToPixelMulti(const std::vector<Vector2f> &groundPoints);

  /**
   * Get observability (sensitivity?) of a given joint for a given camera at a given pose
   */
  Vector3f getSensitivityForJointForCamera(const JOINTS::JOINT &joint, const rawPoseT &jointAngles, const SUPPORT_FOOT &sf,
                                           const std::vector<Vector2f> &grid, const std::vector<float> &baselinePoints,
                                           const SENSOR_NAME &sensorName, bool &observed);

  /**
   * Get observability (sensitivity?) of each joint for a given camera at a given pose
   */
  PoseSensitivity<Vector3f> getSensitivitiesForCamera(const rawPoseT &jointAngles, const SUPPORT_FOOT &sf, const SENSOR_NAME &sensorName);

  /**
   * Get sensitivities for each joint and each sensor
   */
  std::vector<PoseSensitivity<Vector3f>> getSensitivities(const rawPoseT &jointAngles, const SUPPORT_FOOT &sf, const std::vector<SENSOR_NAME> &sensorNames);
};
