#include "NaoProjectionDataProvider.hpp"

#include <Tools/Kinematics/ForwardKinematics.h>
#include <Tools/Kinematics/InverseKinematics.h>

/**
 * This implements the very minimal of what actual SensorDataProvider of Nao
 * codebase does.
 * Also the basics of Projection Module.
 *
 * 1. This class can digest joint angles and return torso to ground, but not
 * using IMU.
 * Instead rely on assumption support foot is firmly on ground.
 */

KinematicMatrix
NaoSensorDataProvider::getTorso2Ground(const KinematicMatrix &supFoot2torso) {
  /// torso2ground
  auto foot2torsoRotM = supFoot2torso.rotM.toRotationMatrix();
  KinematicMatrix rotation(KinematicMatrix::rotY(-asin(foot2torsoRotM(0, 2))) *
                           KinematicMatrix::rotX(asin(foot2torsoRotM(1, 2))));
  KinematicMatrix torso2ground =
      rotation * KinematicMatrix(-supFoot2torso.posV);
  torso2ground.posV.x() = 0;
  torso2ground.posV.y() = 0;
  return torso2ground;
}

KinematicMatrix
NaoSensorDataProvider::getSupportFootMatrix(const std::vector<float> &angles,
                                            const SUPPORT_FOOT &sf) {
  KinematicMatrix supFoot2Torso;
  if (sf == SUPPORT_FOOT::SF_NONE) {
    // TODO catch these exceptions instead of direct printing?
    std::cerr << "None-support foot. Cannot compute cam2gnd" << std::endl;
    throw "None-support foot. Cannot compute cam2gnd";
  }
  if (sf == SUPPORT_FOOT::SF_RIGHT) {
    supFoot2Torso = ForwardKinematics::getRFoot(
        rawPoseT(angles.begin() + JOINTS::R_HIP_YAW_PITCH,
                 angles.begin() + JOINTS::R_ANKLE_ROLL + 1));
  } else {
    supFoot2Torso = ForwardKinematics::getLFoot(
        rawPoseT(angles.begin() + JOINTS::L_HIP_YAW_PITCH,
                 angles.begin() + JOINTS::L_ANKLE_ROLL + 1));
  }

  return supFoot2Torso;
}

void NaoSensorDataProvider::updatedCameraMatrix(
    const std::vector<float> &angles, const SUPPORT_FOOT &sf,
    CameraMatrix &cameraMatrix_, const Camera &camera,
    const bool &useProjectedTorsoAsGroundOrigin) {
  updatedCameraMatrix(angles, getSupportFootMatrix(angles, sf), cameraMatrix_,
                      camera, useProjectedTorsoAsGroundOrigin);
}
/**
 * @param
 * Derived from cycle() of Projection.cpp
 */
void NaoSensorDataProvider::updatedCameraMatrix(
    const std::vector<float> &angles, const KinematicMatrix &supFoot2torso,
    CameraMatrix &cameraMatrix_, const Camera &camera,
    const bool &useProjectedTorsoAsGroundOrigin) {
  KinematicMatrix camera2head_uncalib;
  switch (camera) {
  case Camera::TOP:
    camera2head_uncalib = KinematicMatrix::transZ(63.64) *
                          KinematicMatrix::transX(58.71) *
                          KinematicMatrix::rotY(0.0209);
    break;
  case Camera::BOTTOM:
    camera2head_uncalib = KinematicMatrix::transZ(17.74) *
                          KinematicMatrix::transX(50.71) *
                          KinematicMatrix::rotY(0.6929);
    break;
  }
  cameraMatrix_.camera2torso = getHeadToTorso(angles) * camera2head_uncalib;

  // Differs from original hulks code. assuming the nao's supportfoot dont leave
  // ground, supFootMode is good for grid stuff
  if (useProjectedTorsoAsGroundOrigin) {
    cameraMatrix_.camera2ground =
        getTorso2Ground(supFoot2torso) * cameraMatrix_.camera2torso;
  } else {
    cameraMatrix_.camera2ground =
        supFoot2torso.invert() * cameraMatrix_.camera2torso;
  }
  // divide position by 1000 because we want it in meters but the head matrix
  // buffer stores them in millimeters.
  cameraMatrix_.camera2torso.posV /= 1000.f;
  cameraMatrix_.camera2ground.posV /= 1000.f;
  // do some calculations here because they are needed in other functions that
  // may be called often
  cameraMatrix_.camera2torsoInv = cameraMatrix_.camera2torso.invert();
  cameraMatrix_.camera2groundInv = cameraMatrix_.camera2ground.invert();

  const auto rM = cameraMatrix_.camera2ground.rotM.toRotationMatrix();

  if (rM(2, 2) == 0.f) {
    // Assume that the horizon is above the image.
    cameraMatrix_.horizonA = 0;
    cameraMatrix_.horizonB = 0;
  } else {
    // These formula can be derived from the condition that at the coordinates
    // (x, y) the pixel ray is parallel to the ground.
    cameraMatrix_.horizonA =
        -cameraMatrix_.fc.y() * rM(2, 1) / (cameraMatrix_.fc.x() * rM(2, 2));
    cameraMatrix_.horizonB =
        cameraMatrix_.cc.y() +
        cameraMatrix_.fc.y() *
            (rM(2, 0) +
             cameraMatrix_.cc.x() * rM(2, 1) / cameraMatrix_.fc.x()) /
            rM(2, 2);
  }
  cameraMatrix_.valid = true;
}

std::vector<std::pair<Vector2f, Vector2f>>
NaoSensorDataProvider::getFilteredCorrespondancePairs(
    const VecVector2<float> &baseline,
    const std::vector<std::pair<bool, Vector2f>> &meas) {
  std::vector<std::pair<Vector2f, Vector2f>> output;
  if (baseline.size() == meas.size()) {
    for (size_t i = 0; i < baseline.size(); i++) {
      if (meas[i].first) {
        output.emplace_back(baseline[i], meas[i].second);
      }
    }
  } else {
    std::cout << "baseline and measurement list sizes mismatch";
  }
  //#if DEBUG_CAM_OBS
  //  if (output.size() != baseline.size()) {
  //    std::cout << "diff " << (baseline.size() - output.size()) << std::endl;
  //  }
  //#endif
  return output;
}

VecVector2<float> NaoSensorDataProvider::filterRobot2PixelSets(
    const std::vector<std::pair<bool, Vector2f>> &vec) {
  VecVector2<float> output;
  for (const auto &i : vec) {
    if (i.first) {
      output.push_back(i.second);
    }
  }
  return output;
}
