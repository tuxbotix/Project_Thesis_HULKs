#pragma once

#include <Tools/Kinematics/KinematicMatrix.h>
#include <Data/CameraMatrix.hpp>
#include <Hardware/CameraInterface.hpp>
#include "NaoPoseInfo.hpp"

/**
 * This implements the very minimal of what actual SensorDataProvider of Nao codebase does.
 * Also the basics of Projection Module.
 * 
 * 1. This class can digest joint angles and return torso to ground, but not using IMU.
 * Instead rely on assumption support foot is firmly on ground.
 */

class NaoSensorDataProvider
{
    static KinematicMatrix getTorso2Ground(const KinematicMatrix &supFoot2torso);

    inline static KinematicMatrix getHeadToTorso(const std::vector<float> &angles)
    {
        return ForwardKinematics::getHeadPitch(angles);
    }

  public:
    static KinematicMatrix getSupportFootMatrix(const std::vector<float> &angles, const SUPPORT_FOOT &sf);
    static void updatedCameraMatrix(const std::vector<float> &angles, const SUPPORT_FOOT &sf, CameraMatrix &cameraMatrix_, const Camera &camera);
    /**
     * Derived from cycle() of Projection.cpp
     */
    static void updatedCameraMatrix(const std::vector<float> &angles, const KinematicMatrix &supFoot2torso, CameraMatrix &cameraMatrix_, const Camera &camera);
};
