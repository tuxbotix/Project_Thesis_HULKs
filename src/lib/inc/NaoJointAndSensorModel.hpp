#pragma once

#include <cmath>
#include <limits> // std::numeric_limits

#include "Data/CameraMatrix.hpp"

#include "CalibrationFeatures.hpp"
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
  //  VecVector2<float> basicGrid;

public:
  NaoJointAndSensorModel(
      const Vector2i imSize, const Vector2f fc, const Vector2f cc,
      const Vector2f fov/*,
       const size_t maxGridPointsPerSide, const float gridSpacing*/)
      : jointCalibOffsets(JOINTS::JOINT::JOINTS_MAX, 0.0),
        commandedPose(JOINTS::JOINT::JOINTS_MAX, 0.0),
        realPose(JOINTS::JOINT::JOINTS_MAX, 0.0), imSize(imSize)/*, basicGrid(0)*/ {

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

    //    float x, y;
    //    const int gridSizeHalf = maxGridPointsPerSide / 2;
    //    for (size_t i = 0; i < maxGridPointsPerSide; i++) {
    //      x = ((int)i - gridSizeHalf) * gridSpacing;
    //      for (size_t j = 0; j < maxGridPointsPerSide; j++) {
    //        y = ((int)j - gridSizeHalf) * gridSpacing;
    //        basicGrid.emplace_back(x, y);
    //      }
    //    }
  }

  static float getMaxViewDist() { return NaoJointAndSensorModel::maxViewDist; }

  NaoJointAndSensorModel(const NaoJointAndSensorModelConfig cfg)
      : NaoJointAndSensorModel(cfg.imSize, cfg.fc, cfg.cc, cfg.fov
                               /*,
                               cfg.maxGridPointsPerSide, cfg.gridSpacing*/) {}
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
          std::min(std::min(camMat.getHorizonHeight(0) + 1,
                            camMat.getHorizonHeight(imSize.x() - 1) + 1),
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
    return horizon[static_cast<Eigen::Index>(camName)] >= (imSize.y() - 1);
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
   * @brief projectCameraFOVToGround
   * We go clockwise order (assuming y is downwards and x to right in image
   * plane)
   * @param camName
   * @param imageCornerGroundProjections
   */
  bool projectCameraFOVToGround(
      const Camera &camName,
      std::array<Vector2f, 4> &imageCornerGroundProjections,
      Vector2f &camCenterProjPoint) const {
    auto &camMat = camName == Camera::TOP ? topCamMat : botCamMat;

    // Check if horizon line is above bottom of camera view.
    bool proceed = !isCameraAboveHorizon(camName);
    if (proceed) {
      proceed = projectCamCenterAxisToGround(camName, camCenterProjPoint) &&
                (camCenterProjPoint.norm()) <= getMaxViewDist();
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
      auto &tl = imageCornerGroundProjections[0];
      auto &tr = imageCornerGroundProjections[1];
      auto &br = imageCornerGroundProjections[2];
      auto &bl = imageCornerGroundProjections[3];
      // these will be indexes (0 index)
      Vector2i imLimits = imSize - Vector2i(1, 1);
      float imHeightFraction = imSize.y() * 0.75f;

      // bottom-left
      proceed &= pixelToRobot(camName, Vector2i(0, imLimits.y()), bl);
      // bottom-right
      proceed &= pixelToRobot(camName, imLimits, br);

      // get ground point of image's center right point
      // if this fails, abort as we can't even see half of the image under
      // horizon
      auto leftTopHoriz = camMat.getHorizonHeight(0) + 1;
      auto rightTopHoriz = camMat.getHorizonHeight(imLimits.x()) + 1;
      if (imHeightFraction > leftTopHoriz && imHeightFraction > rightTopHoriz) {
        Vector2f temp = {0.0f, 0.0f};
        Vector2f diff = {0.0f, 0.0f};
        Vector2f diff2 = {0.0f, 0.0f};
        bool cornerSuccess = false;
        // top left
        cornerSuccess = pixelToRobot(camName, Vector2i(0, leftTopHoriz), tl) &&
                        tl.norm() < getMaxViewDist();
        diff = (tl - bl).normalized();
        // center-left
        //        proceed &=
        pixelToRobot(camName, Vector2i(0, imHeightFraction), temp) &&
            temp.norm() < getMaxViewDist();
        // get gradient of line from bl point of image on ground to cl of image
        // on ground
        diff2 = (temp - bl).normalized();
        // if bl to cl and bl to tl are coinciding (+- 10.0 deg divergence)
        // then this is good
        if (!cornerSuccess ||
            std::abs(std::atan2(diff2.y(), diff2.x()) -
                     std::atan2(diff.y(), diff.x())) > 10.0f * TO_RAD_FLT) {
          // truncate the FOV with maxViewDist to get psuedo top-left
          tl = bl + diff2 * std::max(getMaxViewDist(),
                                     (getMaxViewDist() - bl.norm()));
        }

        // top right
        cornerSuccess =
            pixelToRobot(camName, Vector2i(imLimits.x(), rightTopHoriz), tr) &&
            tr.norm() < getMaxViewDist();
        diff = (tr - br).normalized();
        // center-right
        //        proceed &=
        pixelToRobot(camName, Vector2i(imLimits.x(), imHeightFraction), temp) &&
            temp.norm() < getMaxViewDist();
        // get gradient of line from br point of image on ground to cr of image
        // on ground
        diff2 = (temp - br).normalized();
        // if br to cr and br to tr are coinciding (+- 10.0 deg divergence)
        // then this is good
        if (!cornerSuccess ||
            std::abs(std::atan2(diff2.y(), diff2.x()) -
                     std::atan2(diff.y(), diff.x())) > 10.0f * TO_RAD_FLT) {
          // truncate the FOV with maxViewDist to get psuedo top-left
          tr = br + diff2 * std::max(getMaxViewDist(),
                                     (getMaxViewDist() - br.norm()));
        }
      } else {
        proceed = false;
      }
    }
    return proceed;
  }
  const VecVector2<float> getFilteredCalibFeatures(
      const Camera &camName,
      const std::vector<CalibrationFeatures::CalibrationFeaturePtr<float>>
          &calibrationFeaturePtrs,
      bool &success) const {
    VecVector2<float> output;
    Vector2f robotCoords = {0, 0};
    Vector2f camCenterProjPoint = {0, 0};
    float cornerProjMaxDist = 0; // max distance to corners from center.

    std::array<Vector2f, 4> imageCornerGroundProjections = {};
    if (projectCameraFOVToGround(camName, imageCornerGroundProjections,
                                 camCenterProjPoint)) {
      for (const auto &val : imageCornerGroundProjections) {
        float dist = (camCenterProjPoint - val).norm();
        if (dist > cornerProjMaxDist) {
          cornerProjMaxDist = dist;
        }
      }
      for (const auto &feature : calibrationFeaturePtrs) {
        float theta = std::atan2(robotCoords.y(), robotCoords.x());
        feature->updatePointPose(robotCoords, theta);
        auto basicGrid = feature->getGroundPoints();
        for (const auto &i : basicGrid) {
          if ((camCenterProjPoint - i).norm() <= cornerProjMaxDist) {
            output.push_back(i);
          }
        }
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
    success = (output.size() > 0);
    return output;
  }
  //  /**
  //   * Get 3D grid with given grid spacing relative to robot's ground coord.
  //   * Obviously, camera matrix must be updated before calling this.
  //   * 1. This has a hard range limit of 2m
  //   */
  //  // TODO  Make this private.
  //  const VecVector2<float> getGroundGrid(const Camera &camName,
  //                                        bool &success) const {
  //    VecVector2<float> output;
  //    Vector2f robotCoords = {0, 0};
  //    Vector2f camCenterProjPoint = {0, 0};
  //    float cornerProjMaxDist = 0; // max distance to corners from center.

  //    std::array<Vector2f, 4> imageCornerGroundProjections = {};
  //    if (projectCameraFOVToGround(camName, imageCornerGroundProjections,
  //                                 camCenterProjPoint)) {
  //      for (const auto &val : imageCornerGroundProjections) {
  //        float dist = (camCenterProjPoint - val).norm();
  //        if (dist > cornerProjMaxDist) {
  //          cornerProjMaxDist = dist;
  //        }
  //      }

  //      for (const auto &feature : calibrationFeatures) {
  //        float theta = std::atan2(robotCoords.y(), robotCoords.x());
  //        auto basicGrid = feature.getGroundPoints(robotCoords, theta);
  //        for (const auto &i : basicGrid) {
  //          if ((camCenterProjPoint - i).norm() <= cornerProjMaxDist) {
  //            output.push_back(i);
  //          }
  //        }
  //      }
  //    }
  //#if DEBUG_JOINT_AND_SENS
  //    else {
  //      std::lock_guard<std::mutex> lock(utils::mtx_cout_);
  //      std::cout << "CamCenterRayToGround: " << camCenterProjPoint.norm() <<
  //      " "
  //                << camCenterProjPoint.x() << ", " << camCenterProjPoint.y()
  //                << std::endl;
  //      std::cout << "Too far view dist!" << std::endl;
  //    }
  //#endif
  //    success = (output.size() > 0);
  //    return output;
  //  }
};
