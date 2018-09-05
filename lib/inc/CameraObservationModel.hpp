#pragma once

#include <limits> // std::numeric_limits
#include <cmath>

#include "Data/CameraMatrix.hpp"

#include "ObservationModel.hpp"
#include "NaoProjectionDataProvider.hpp"

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

  std::vector<Vector2f> basicGrid;

  Vector2i imSize;
  // float gridSpacing;
  const float deltaThetaCorse;
  const float deltaThetaFine;
  int horizon;

  // const Vector3f dimensionMax;
  CameraMatrix camMat;

  /**
   * @brief cameraToPixel transforms camera coordinates to pixel coordinates
   * @param cameraCoordinates the camera coordinates
   * @param pixel_coordinates the result is stored here
   * @return whether the transformation was successful
   */
  bool cameraToPixelFlt(const Vector3f &cameraCoordinates, Vector2f &pixel_coordinates) const
  {
    // A position behind the camera cannot be transformed to pixel coordinates as it does not
    // intersect the image plane.
    if (cameraCoordinates.x() <= 0.f)
    {
      return false;
    }
    // pinhole projection
    pixel_coordinates.x() = camMat.cc.x() - camMat.fc.x() * cameraCoordinates.y() / cameraCoordinates.x();
    pixel_coordinates.y() = camMat.cc.y() - camMat.fc.y() * cameraCoordinates.z() / cameraCoordinates.x();
    return true;
  }

  /**
   * @brief robotToPixel calculates the pixel coordinates of a given point (on ground) in robot
   * coordinates
   * @param robotCoordinates coordinates in the plane
   * @param pixel_coordinates the result is stored here
   * @return whether the transformation was successful
   */
  bool robotToPixelFlt(const Vector2f &robotCoordinates, Vector2f &pixel_coordinates,
                       const KinematicMatrix &cam2ground_inv) const
  {
    // calculate camera coordinates from robot coordinates
    Vector3f cameraCoordinates(cam2ground_inv *
                               Vector3f(robotCoordinates.x(), robotCoordinates.y(), 0));
    // do pinhole projection
    return cameraToPixelFlt(cameraCoordinates, pixel_coordinates);
  }
  bool robotToPixelFlt(const Vector2f &robotCoordinates, Vector2f &pixel_coordinates) const
  {
    return robotToPixelFlt(robotCoordinates, pixel_coordinates, camMat.camera2groundInv);
  }

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
  void updateState(const rawPoseT &jointAngles, const SUPPORT_FOOT &sf, const SENSOR_NAME &sensorNames);

  /**
   * Get 3D grid with given grid spacing relative to robot's ground coord.
   * Obviously, camera matrix must be updated before calling this.
   * 1. This has a hard range limit of 2m
   */
  // TODO  Make this private.
  std::vector<Vector2f> getGroundGrid(bool &success);

  /**
   * Filter correspondance pairs
   */
  std::vector<std::pair<Vector2f, Vector2f>> getFilteredCorrespondancePairs(const std::vector<Vector2f> &baseline,
                                                                            const std::vector<std::pair<bool, Vector2f>> &meas) const;

  /**
   * Filter out bad robot2Pixel converts
   */
  std::vector<Vector2f> filterRobot2PixelSets(const std::vector<std::pair<bool, Vector2f>> &vec) const;

  /**
   * Robot to pixel, multiple point support
   */
  std::vector<std::pair<bool, Vector2f>> robotToPixelMulti(const std::vector<Vector2f> &groundPoints) const;

  /**
   * Get observability (sensitivity?) of a given joint for a given camera at a given pose
   */
  Vector3f getSensitivityForJointForCamera(const JOINTS::JOINT &joint, const rawPoseT &jointAngles, const SUPPORT_FOOT &sf,
                                           const std::vector<Vector2f> &grid, const std::vector<Vector2f> &baselinePoints,
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
