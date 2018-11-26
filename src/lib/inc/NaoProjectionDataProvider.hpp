#pragma once

#include "NaoPoseInfo.hpp"
#include <Data/CameraMatrix.hpp>
#include <Hardware/CameraInterface.hpp>
#include <Tools/Kinematics/KinematicMatrix.h>

/**
 * This implements the very minimal of what actual SensorDataProvider of Nao
 * codebase does.
 * Also the basics of Projection Module.
 *
 * 1. This class can digest joint angles and return torso to ground, but not
 * using IMU.
 * Instead rely on assumption support foot is firmly on ground.
 */

class NaoSensorDataProvider {
  static KinematicMatrix getTorso2Ground(const KinematicMatrix &supFoot2torso);

  inline static KinematicMatrix
  getHeadToTorso(const std::vector<float> &angles) {
    return ForwardKinematics::getHeadPitch(angles);
  }

public:
  static KinematicMatrix getSupportFootMatrix(const std::vector<float> &angles,
                                              const SUPPORT_FOOT &sf);
  static void
  updatedCameraMatrix(const std::vector<float> &angles, const SUPPORT_FOOT &sf,
                      CameraMatrix &cameraMatrix_, const Camera &camera,
                      const bool &useProjectedTorsoAsGroundOrigin = true);
  /**
   * @param useProjectedTorsoAsGroundOrigin - if true will use standard hulks
   * method of torso projection to ground. Else it'll take ground's origin as
   * point of support foot on ground.
   * Derived from cycle() of Projection.cpp
   */
  static void
  updatedCameraMatrix(const std::vector<float> &angles,
                      const KinematicMatrix &supFoot2torso,
                      CameraMatrix &cameraMatrix_, const Camera &camera,
                      const bool &useProjectedTorsoAsGroundOrigin = true);

  /**
   * Filter correspondance pairs
   */
  static std::vector<std::pair<Vector2f, Vector2f>>
  getFilteredCorrespondancePairs(
      const VecVector2<float> &baseline,
      const std::vector<std::pair<bool, Vector2f>> &meas);

  /**
   * Filter out bad robot2Pixel converts
   */
  static VecVector2<float>
  filterRobot2PixelSets(const std::vector<std::pair<bool, Vector2f>> &vec);
};
