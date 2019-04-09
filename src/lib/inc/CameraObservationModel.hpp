#pragma once

#include <cmath>
#include <limits> // std::numeric_limits

#include "Data/CameraMatrix.hpp"

#include "CalibrationFeatures.hpp"
#include "NaoJointAndSensorModel.hpp"
#include "NaoProjectionDataProvider.hpp"
#include "ObservationModel.hpp"

#ifndef DEBUG_CAM_OBS
#define DEBUG_CAM_OBS 0
#endif
/**
 * To be precise, this give sensitivity of camera observation :P
 */
class CameraObservationModel : public ObservationModel {
private:
  /**
   * To be precise, this give sensitivity of camera observation :P
   */
  typedef int16_t sensitivityOutputType;
  static const std::string name;

  const uint64_t maxValPerDim;
  Vector2i imSize;
  const float deltaThetaCorse;
  const float deltaThetaFine;

  NaoJointAndSensorModel naoJointSensorModel;
  std::vector<CalibrationFeatures::CalibrationFeaturePtr<float>>
      calibrationFeaturePtrs;

public:
  virtual ~CameraObservationModel() {}
  // friend class ObservationSensitivity;
  CameraObservationModel(const Vector2i &imSize, const Vector2f &fc,
                         const Vector2f &cc, const Vector2f &fov,
                         uint64_t dimensionExtremum,
                         const size_t &maxGridPointsPerSide,
                         const float &gridSpacing);

  std::string getName() { return name; }

  /**
   * Update the state of the robot.
   */
  void updateState(const rawPoseT &jointAngles, const SUPPORT_FOOT &sf,
                   const Camera &sensorNames);

  /**
   * Get observability (sensitivity?) of a given joint for a given camera at a
   * given pose
   */
  const Vector3f getSensitivityForJointForCamera(
      const JOINTS::JOINT &joint, const rawPoseT &jointAngles,
      const SUPPORT_FOOT &sf, const VecVector2<float> &grid,
      const VecVector2<float> &baselinePoints, const Camera &sensorName,
      bool &observed);

  /**
   * Get observability (sensitivity?) of each joint for a given camera at a
   * given pose
   */
  const PoseSensitivity<Vector3f>
  getSensitivitiesForCamera(const rawPoseT &jointAngles, const SUPPORT_FOOT &sf,
                            const SENSOR_NAME &sensorName);

  /**
   * Get sensitivities for each joint and each sensor
   */
  const std::vector<PoseSensitivity<Vector3f>>
  getSensitivities(const rawPoseT &jointAngles, const SUPPORT_FOOT &sf,
                   const std::vector<SENSOR_NAME> &sensorNames);
};
