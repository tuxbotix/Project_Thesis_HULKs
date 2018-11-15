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

public:
  NaoJointAndSensorModel(const Vector2i &imSize, const Vector2f &fc,
                         const Vector2f &cc, const Vector2f &fov)
      : imSize(imSize), 
      jointCalibOffsets(JOINTS::JOINT::JOINTS_MAX, 0.0),
      commandedPose(JOINTS::JOINT::JOINTS_MAX, 0.0),
      realPose (JOINTS::JOINT::JOINTS_MAX, 0.0){

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
    NaoSensorDataProvider::updatedCameraMatrix(realPose, sf, camMat, camName);
    // update default horizon
    horizon[(int)camName] =
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
};
