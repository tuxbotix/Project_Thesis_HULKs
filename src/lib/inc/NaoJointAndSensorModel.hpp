#pragma once

#include <cmath>
#include <limits> // std::numeric_limits

#include "Data/CameraMatrix.hpp"

#include "NaoProjectionDataProvider.hpp"
#include "ObservationModel.hpp"
#include "utils.hpp"

#ifndef DEBUG_JOINT_AND_SENS
#define DEBUG_JOINT_AND_SENS 0
#endif

struct NaoJointAndSensorModelConfig {
  Vector2i imSize;
  Vector2f fc;
  Vector2f cc;
  Vector2f fov;
  size_t maxGridPointsPerSide;
  float gridSpacing;
};

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
  // updateState(Poses::getPose(Poses::READY), SUPPORT_FOOT::SF_LEFT,
  // SENSOR_NAME::TOP_CAMERA);
  VecVector2<float> basicGrid;

public:
  NaoJointAndSensorModel(const Vector2i imSize, const Vector2f fc,
                         const Vector2f cc, const Vector2f fov,
                         const size_t maxGridPointsPerSide,
                         const float gridSpacing)
      : jointCalibOffsets(JOINTS::JOINT::JOINTS_MAX, 0.0),
        commandedPose(JOINTS::JOINT::JOINTS_MAX, 0.0),
        realPose(JOINTS::JOINT::JOINTS_MAX, 0.0), imSize(imSize), basicGrid(0) {

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
    for (size_t i = 0; i < maxGridPointsPerSide; i++) {
      x = ((int)i - gridSizeHalf) * gridSpacing;
      for (size_t j = 0; j < maxGridPointsPerSide; j++) {
        y = ((int)j - gridSizeHalf) * gridSpacing;
        basicGrid.emplace_back(x, y);
      }
    }
  }

  NaoJointAndSensorModel(const NaoJointAndSensorModelConfig cfg)
      : NaoJointAndSensorModel(cfg.imSize, cfg.fc, cfg.cc, cfg.fov,
                               cfg.maxGridPointsPerSide, cfg.gridSpacing) {}
  ~NaoJointAndSensorModel() {}

  const Vector2i getImSize() const { return imSize; }

  void setPoseCamUpdateOnly(const rawPoseT &jointAngles, const SUPPORT_FOOT &sf,
                            const std::vector<Camera> &camNames) {
    commandedPose = jointAngles;
    // see JointCalibrationProvider? .cpp
    for (size_t i = 0; i < JOINTS::JOINT::JOINTS_MAX; i++) {
      realPose[i] = commandedPose[i] + jointCalibOffsets[i];
    }

    for (const auto &camName : camNames) {
      auto &camMat = camName == Camera::TOP ? topCamMat : botCamMat;
      // update sensor model based on the REAL pose!!

      // Always use supportFootOrigin policy
      NaoSensorDataProvider::updatedCameraMatrix(realPose, sf, camMat, camName,
                                                 false);
      // update default horizon
      horizon[static_cast<int>(camName)] =
          std::min(std::min(camMat.getHorizonHeight(0),
                            camMat.getHorizonHeight(imSize.x() - 1)),
                   imSize.y() - 1);
    }
  }

  void setPose(const rawPoseT &jointAngles, const SUPPORT_FOOT &sf) {
    setPoseCamUpdateOnly(jointAngles, sf, {Camera::TOP, Camera::BOTTOM});
  }

  const rawPoseT getRawPose() const {
    return commandedPose;
  } // for general usage

  const rawPoseT getRealPose() const {
    return realPose;
  } // used for evaluation only

  void setCalibValues(const rawPoseT &calibVals) {
    if (calibVals.size() < JOINTS::JOINT::JOINTS_MAX) {
      std::cerr << "calibVals with incorrect length " << calibVals.size()
                << std::endl;
      throw "calibVals with incorrect length";
    }
    for (size_t i = 0; i < JOINTS::JOINT::JOINTS_MAX; i++) {
      jointCalibOffsets[i] = calibVals[i];
    }
  }

  rawPoseT getCalibValues() { return jointCalibOffsets; }

  bool pixelToRobot(const Camera &camName, const Vector2i &pixelCoords,
                    Vector2f &robotCoords) const {
    auto &camMat = camName == Camera::TOP ? topCamMat : botCamMat;
    return camMat.pixelToRobot(pixelCoords, robotCoords);
  }

  bool projectCamCenterAxisToGround(const Camera &camName,
                                    Vector2f &robotCoords) const {
    auto &camMat = camName == Camera::TOP ? topCamMat : botCamMat;

    if (camMat.pixelToRobot(camMat.cc.cast<int>(), robotCoords)) {
      return true;
    } else {
      std::cout << robotCoords << camMat.cc << std::endl;
      return false;
    }
  }

  bool isCameraAboveHorizon(const Camera &camName) const {
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
    pixel_coordinates.x() =
        camMat.cc.x() -
        camMat.fc.x() * cameraCoordinates.y() / cameraCoordinates.x();
    pixel_coordinates.y() =
        camMat.cc.y() -
        camMat.fc.y() * cameraCoordinates.z() / cameraCoordinates.x();
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
  const VecVector2<float> getGroundGrid(const Camera &camName,
                                        bool &success) const {
    VecVector2<float> output;
    Vector2f robotCoords = {0, 0};
    Vector2f camCenterProjPoint = {0, 0};
    float cornerProjMaxDist = 0; // max distance to corners from center.

    // Check if horizon line is above bottom of camera view.
    bool proceed = !isCameraAboveHorizon(camName);
    if (proceed) {
      proceed = projectCamCenterAxisToGround(camName, camCenterProjPoint) &&
                (camCenterProjPoint.norm()) <= maxViewDist;
    }
#if DEBUG_JOINT_AND_SENS
    else {
      std::lock_guard<std::mutex> lock(utils::mtx_cout_);
      auto &camMat = camName == Camera::TOP ? topCamMat : botCamMat;
      std::cout << "HorizA" << camMat.horizonA << " horizB " << camMat.horizonB
                << std::endl;
      std::cout << " Out of horizon!" << std::endl;
      std::cout << camMat.pixelToRobot(Vector2i(0, 0), robotCoords) << "("
                << robotCoords.x() << ", " << robotCoords.y() << ")\t";
      std::cout << camMat.pixelToRobot(Vector2i(imSize.x(), 0), robotCoords)
                << "(" << robotCoords.x() << ", " << robotCoords.y() << ")"
                << std::endl;
      std::cout << camMat.pixelToRobot(Vector2i(0, imSize.y()), robotCoords)
                << "(" << robotCoords.x() << ", " << robotCoords.y() << ")\t";
      std::cout << camMat.pixelToRobot(imSize, robotCoords) << "("
                << robotCoords.x() << ", " << robotCoords.y() << ")"
                << std::endl;
    }
#endif
    if (proceed) {
      std::array<Vector2f, 4> imageCornerGroundProjections = {};
      pixelToRobot(camName, Vector2i(0, 0), imageCornerGroundProjections[0]);
      pixelToRobot(camName, Vector2i(0, imSize.y()),
                   imageCornerGroundProjections[1]);
      pixelToRobot(camName, Vector2i(imSize.x(), 0),
                   imageCornerGroundProjections[2]);
      pixelToRobot(camName, imSize, imageCornerGroundProjections[3]);

      for (const auto &val : imageCornerGroundProjections) {
        float dist = (camCenterProjPoint - val).norm();
        if (dist > cornerProjMaxDist) {
          cornerProjMaxDist = dist;
        }
      }

      Vector2f tempCoord;
      // Eigen::Translation<float, 2> trans(tempCoord.x(), tempCoord.y());
      float theta = std::atan2(robotCoords.y(), robotCoords.x());
      Eigen::Rotation2D<float> rot2(theta);
      for (const auto &i : basicGrid) {
        tempCoord = robotCoords + (rot2 * i);
        // if this point might be in image's viewpoint.
        if ((camCenterProjPoint - tempCoord).norm() <= cornerProjMaxDist) {
          output.push_back(tempCoord);
        }
        //        std::cout << (camCenterProjPoint - tempCoord).norm() << "<="
        //        <<  cornerProjMaxDist << " = " << ((camCenterProjPoint -
        //        tempCoord).norm() <= cornerProjMaxDist) << std::endl;
      }
    }
#if DEBUG_JOINT_AND_SENS
    else {
      std::lock_guard<std::mutex> lock(utils::mtx_cout_);
      std::cout << "CamCenterRayToGround: " << camCenterProjPoint.norm() << " "
                << camCenterProjPoint.x() << ", " << camCenterProjPoint.y()
                << std::endl;
      std::cout << "Too far view dist!" << std::endl;
    }
#endif
    success = proceed && (output.size() > 0);
    return output;
  }
};
