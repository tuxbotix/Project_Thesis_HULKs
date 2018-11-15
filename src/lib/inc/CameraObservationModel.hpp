#pragma once

#include <limits> // std::numeric_limits
#include <cmath>

#include "Data/CameraMatrix.hpp"

#include "ObservationModel.hpp"
#include "NaoProjectionDataProvider.hpp"
#include "NaoJointAndSensorModel.hpp"

#ifndef DEBUG_CAM_OBS
#define DEBUG_CAM_OBS 0
#endif
/**
 * To be precise, this give sensitivity of camera observation :P
 */
class CameraObservationModel : public ObservationModel
{
private:
  /**
 * To be precise, this give sensitivity of camera observation :P
 */
  typedef int16_t sensitivityOutputType;
  static const std::string name;
  static constexpr float maxViewDist = 3.0;
  // we scale the output to a signed integer occupying the below byte count.
  // ie: to fit
  // static const uint8_t bytesPerDim = sizeof(sensitivityOutputType);
  const uint64_t maxValPerDim; // = std::numeric_limits<sensitivityOutputType>::max(); // max per side*, so - 38768

  // const size_t maxGridPointsPerSide;

  VecVector2<float> basicGrid;

  Vector2i imSize;
  // float gridSpacing;
  const float deltaThetaCorse;
  const float deltaThetaFine;

  // const Vector3f dimensionMax;

  // TODO make this private??
public:
  NaoJointAndSensorModel naoJointSensorModel;

public:
  ~CameraObservationModel() {}
  // friend class ObservationSensitivity;
  CameraObservationModel(const Vector2i &imSize, const Vector2f &fc, const Vector2f &cc, const Vector2f &fov,
                         uint64_t dimensionExtremum, const size_t &maxGridPointsPerSide, const float &gridSpacing);

  std::string getName()
  {
    return name;
  }

  /**
   * Update the state of the robot.
   */
  void updateState(const rawPoseT &jointAngles, const SUPPORT_FOOT &sf, const Camera &sensorNames);

  /**
   * Get 3D grid with given grid spacing relative to robot's ground coord.
   * Obviously, camera matrix must be updated before calling this.
   * 1. This has a hard range limit of 2m
   */
  // TODO  Make this private.
  VecVector2<float> getGroundGrid(const Camera &camName, bool &success);

  /**
   * Filter correspondance pairs
   */
  std::vector<std::pair<Vector2f, Vector2f>> getFilteredCorrespondancePairs(const VecVector2<float> &baseline,
                                                                            const std::vector<std::pair<bool, Vector2f>> &meas) const;

  /**
   * Filter out bad robot2Pixel converts
   */
  VecVector2<float> filterRobot2PixelSets(const std::vector<std::pair<bool, Vector2f>> &vec) const;

  /**
   * Robot to pixel, multiple point support
   */
  std::vector<std::pair<bool, Vector2f>> robotToPixelMulti(const VecVector2<float> &groundPoints) const;

  /**
   * Get observability (sensitivity?) of a given joint for a given camera at a given pose
   */
  Vector3f getSensitivityForJointForCamera(const JOINTS::JOINT &joint, const rawPoseT &jointAngles, const SUPPORT_FOOT &sf,
                                           const VecVector2<float> &grid, const VecVector2<float> &baselinePoints,
                                           const Camera &sensorName, bool &observed);

  /**
   * Get observability (sensitivity?) of each joint for a given camera at a given pose
   */
  PoseSensitivity<Vector3f> getSensitivitiesForCamera(const rawPoseT &jointAngles, const SUPPORT_FOOT &sf, const SENSOR_NAME &sensorName);

  /**
   * Get sensitivities for each joint and each sensor
   */
  std::vector<PoseSensitivity<Vector3f>> getSensitivities(const rawPoseT &jointAngles, const SUPPORT_FOOT &sf, const std::vector<SENSOR_NAME> &sensorNames);
};
