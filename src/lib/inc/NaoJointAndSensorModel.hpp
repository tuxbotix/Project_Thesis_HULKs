#pragma once

#include <cmath>
#include <limits> // std::numeric_limits

#include "Data/CameraMatrix.hpp"

#include "NaoProjectionDataProvider.hpp"
#include "ObservationModel.hpp"

class NaoJointAndSensorModel {
private:
  rawPoseT jointCalibOffsets; // inverse of error
  rawPoseT commandedPose;
  rawPoseT realPose;

  CameraMatrix topCamMat;
  CameraMatrix botCamMat;

  Vector2i imSize;
  Vector2i horizon;
  static constexpr float maxViewDist = 3.0;

  // Default state, ready pose and top camera with double foot.
  // updateState(Poses::getPose(Poses::READY), SUPPORT_FOOT::SF_LEFT, SENSOR_NAME::TOP_CAMERA);
  VecVector2<float> basicGrid;

public:
  NaoJointAndSensorModel(const Vector2i &imSize, const Vector2f &fc,
                         const Vector2f &cc, const Vector2f &fov,  const size_t &maxGridPointsPerSide, const float &gridSpacing)
      : imSize(imSize), 
      jointCalibOffsets(JOINTS::JOINT::JOINTS_MAX, 0.0),
      commandedPose(JOINTS::JOINT::JOINTS_MAX, 0.0),
      realPose (JOINTS::JOINT::JOINTS_MAX, 0.0),
       basicGrid(0){

    topCamMat.fc = fc;
    topCamMat.fc.x() *= imSize.x();
    topCamMat.fc.y() *= imSize.y();
    topCamMat.cc = cc;
    topCamMat.cc.x() *= imSize.x();
    topCamMat.cc.y() *= imSize.y();
    topCamMat.fov = fov;
    botCamMat.fc = fc;
    botCamMat.fc.x() *= imSize.x();
    botCamMat.fc.y() *= imSize.y();
    botCamMat.cc = cc;
    botCamMat.cc.x() *= imSize.x();
    botCamMat.cc.y() *= imSize.y();
    botCamMat.fov = fov;

    float x, y;
    const int gridSizeHalf = maxGridPointsPerSide / 2;
    for (size_t i = 0; i < maxGridPointsPerSide; i++)
    {
      x = ((int)i - gridSizeHalf) * gridSpacing;
      for (size_t j = 0; j < maxGridPointsPerSide; j++)
      {
        y = ((int)j - gridSizeHalf) * gridSpacing;
        basicGrid.emplace_back(x, y);
      }
    }
  }
  ~NaoJointAndSensorModel() {}

  void setPoseCamUpdateOnly(const rawPoseT &jointAngles, const SUPPORT_FOOT &sf,
                            const Camera &camName) {
    commandedPose = jointAngles;
    // see JointCalibrationProvider? .cpp
    for (size_t i = 0; i < JOINTS::JOINT::JOINTS_MAX; i++) {
      realPose[i] = commandedPose[i] + jointCalibOffsets[i];
    }

    auto &camMat = camName == Camera::TOP ? topCamMat : botCamMat;
    // update sensor model based on the REAL pose!!

    // Always use supportFootOrigin policy
    NaoSensorDataProvider::updatedCameraMatrix(realPose, sf, camMat, camName, false);
    // update default horizon
    horizon[static_cast<int>(camName)] =
        std::min(std::min(camMat.getHorizonHeight(0),
                          camMat.getHorizonHeight(imSize.x() - 1)),
                 imSize.y() - 1);
  }

  void setPose(const rawPoseT &jointAngles, const SUPPORT_FOOT &sf) {
    setPoseCamUpdateOnly(jointAngles, sf, Camera::TOP);
    setPoseCamUpdateOnly(jointAngles, sf, Camera::BOTTOM);
  }

  rawPoseT getRawPose() { return commandedPose; } // for general usage

  rawPoseT getRealPose() { return realPose; } // used for evaluation only

  void setCalibValues(const rawPoseT &calibVals) {
    if (calibVals.size() < JOINTS::JOINT::JOINTS_MAX) {
      return;
    }
    for (size_t i = 0; i < JOINTS::JOINT::JOINTS_MAX; i++) {
      jointCalibOffsets[i] = calibVals[i];
    }
  }

  rawPoseT getCalibValues() { return jointCalibOffsets; }

  bool projectCamCenterAxisToGround(const Camera &camName,
                                    Vector2f &robotCoords) {
    auto &camMat = camName == Camera::TOP ? topCamMat : botCamMat;
    return camMat.pixelToRobot(camMat.cc.cast<int>(), robotCoords);
  }

  bool isCameraAboveHorizon(const Camera &camName) {
    return horizon[(int)camName] >= (imSize.y() - 1);
  }
  /**
   * @brief cameraToPixel transforms camera coordinates to pixel coordinates
   * @param cameraCoordinates the camera coordinates
   * @param pixel_coordinates the result is stored here
   * @return whether the transformation was successful
   */
  bool cameraToPixelFlt(const Camera &camera, const Vector3f &cameraCoordinates,
                        Vector2f &pixel_coordinates) const {

    auto &camMat = camera == Camera::TOP ? topCamMat : botCamMat;
    // A position behind the camera cannot be transformed to pixel coordinates
    // as it does not intersect the image plane.
    if (cameraCoordinates.x() <= 0.f) {
      return false;
    }
    // pinhole projection
    pixel_coordinates.x() = camMat.cc.x() - camMat.fc.x() *
                                                cameraCoordinates.y() /
                                                cameraCoordinates.x();
    pixel_coordinates.y() = camMat.cc.y() - camMat.fc.y() *
                                                cameraCoordinates.z() /
                                                cameraCoordinates.x();
    return true;
  }

  /**
   * @brief robotToPixel calculates the pixel coordinates of a given point (on
   * ground) in robot coordinates
   * @param robotCoordinates coordinates in the plane
   * @param pixel_coordinates the result is stored here
   * @return whether the transformation was successful
   */
  bool robotToPixelFlt(const Camera &camera, const Vector2f &robotCoordinates,
                       Vector2f &pixel_coordinates,
                       const KinematicMatrix &cam2ground_inv) const {

    // calculate camera coordinates from robot coordinates
    Vector3f cameraCoordinates(cam2ground_inv * Vector3f(robotCoordinates.x(),
                                                         robotCoordinates.y(),
                                                         0));
    // do pinhole projection
    return cameraToPixelFlt(camera, cameraCoordinates, pixel_coordinates);
  }

  bool robotToPixelFlt(const Camera &camera, const Vector2f &robotCoordinates,
                       Vector2f &pixel_coordinates) const {

    auto &camMat = camera == Camera::TOP ? topCamMat : botCamMat;

    return robotToPixelFlt(camera, robotCoordinates, pixel_coordinates,
                           camMat.camera2groundInv);
  }

  /**
   * Robot to pixel, multiple point support
   */
  std::vector<std::pair<bool, Vector2f>>
  robotToPixelMulti(const Camera &camName,
                    const VecVector2<float> &groundPoints) const {
    std::vector<std::pair<bool, Vector2f>> output;
    std::pair<bool, Vector2f> val;

    Vector2f point(0.0, 0.0);

    for (const auto &i : groundPoints) {
      val.first = robotToPixelFlt(camName, i, point);
      if (!val.first || point.x() > imSize.x() || point.y() > imSize.y() ||
          point.x() < 0 || point.y() < 0) {
        val.first = false;
        val.second = Vector2f(0, 0);
      } else {
        val.second = point;
      }
      output.push_back(val);
    }
    return output;
  }


  /**
   * Get 3D grid with given grid spacing relative to robot's ground coord.
   * Obviously, camera matrix must be updated before calling this.
   * 1. This has a hard range limit of 2m
   */
  // TODO  Make this private.
  VecVector2<float> getGroundGrid(const Camera &camName, bool &success)
  {
    VecVector2<float> output;
    Vector2f robotCoords;

    // Check if horizon line is above bottom of camera view.
    bool proceed = !isCameraAboveHorizon(camName);
    if (proceed)
    {
      proceed = projectCamCenterAxisToGround(camName, robotCoords) && robotCoords.norm() <= maxViewDist;
    }
  #if DEBUG_CAM_OBS
    else
    {
      auto &camMat = camName == Camera::TOP ? topCamMat : botCamMat;
      std::cout << "HorizA" << camMat.horizonA << " horizB " << camMat.horizonB << std::endl;
      std::cout << " Out of horizon!" << std::endl;
      std::cout << camMat.pixelToRobot(Vector2i(0, 0), robotCoords) << "(" << robotCoords.x() << ", " << robotCoords.y() << ")\t";
      std::cout << camMat.pixelToRobot(Vector2i(imSize.x(), 0), robotCoords) << "(" << robotCoords.x() << ", " << robotCoords.y() << ")" << std::endl;
      std::cout << camMat.pixelToRobot(Vector2i(0, imSize.y()), robotCoords) << "(" << robotCoords.x() << ", " << robotCoords.y() << ")\t";
      std::cout << camMat.pixelToRobot(imSize, robotCoords) << "(" << robotCoords.x() << ", " << robotCoords.y() << ")" << std::endl;
    }
  #endif
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
  #if DEBUG_CAM_OBS
    else
    {
      std::cout << "CamCenterRayToGround: " << robotCoords.norm() << " " << robotCoords.x() << ", " << robotCoords.y() << std::endl;
      std::cout << "Too far view dist!" << std::endl;
    }
  #endif
    success = proceed;
    return output;
  }
};
