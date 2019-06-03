/**
 * This is a small set of tests to verify if things work as they should.
 *
 */

#include <algorithm>
#include <fstream>
#include <iostream>
#include <numeric>
#include <thread>

#include <Data/CameraMatrix.hpp>
#include <Modules/Configuration/Configuration.h>
#include <Modules/Configuration/UnixSocketConfig.hpp>
#include <Modules/NaoProvider.h>
#include <Modules/Poses.h>

#include <Hardware/RobotInterface.hpp>
#include <Tools/Kinematics/KinematicMatrix.h>
#include <Tools/Storage/Image.hpp>
#include <Tools/Storage/Image422.hpp>

#include "TUHHMin.hpp"

#define PRINT_EXCEPT 1

#include "MiniConfigHandle.hpp"
#include "NaoPoseInfo.hpp"
#include "NaoStability.hpp"
#include "NaoTorsoPose.hpp"
#define DEBUG_CAM_OBS 1
#include "ObservationSensitivityProvider.hpp"

class MinMaxInc {
public:
  Vector2f minHeadYawPitch;
  Vector2f maxHeadYawPitch;
  Vector2f incHeadYawPitch;
  std::array<Vector3f, 4> min = {};
  std::array<Vector3f, 4> max = {};
  std::array<Vector3f, 4> inc = {};

  inline void getHeadYawLimits(dataT &minLim, dataT &maxLim,
                               dataT &increment) const // 8
  {
    // Instead of the full range, i'll limit..
    minLim = minHeadYawPitch[0];
    maxLim = maxHeadYawPitch[0];
    increment = incHeadYawPitch[0];
  }
  inline void getHeadPitchLimits(const dataT &headYaw, dataT &minLim,
                                 dataT &maxLim, dataT &increment) const // 19
  {
    minLim = std::max(NaoProvider::minRangeHeadPitch(headYaw * TO_RAD), 0.0f) /
             TO_RAD;
    maxLim = NaoProvider::maxRangeHeadPitch(headYaw * TO_RAD) / TO_RAD;
    increment = incHeadYawPitch[1];
  }

  inline void getLimits(const paramNameT &paramIndex, dataT &minLim,
                        dataT &maxLim, dataT &increment) const {
    if (paramIndex >= static_cast<PARAMS>(PARAMS::P_MAX) || paramIndex < 0) {
      minLim = 0;
      maxLim = 0;
      increment = 0;

      std::cout << "param index out of bounds" << std::endl;
      return;
    } else if (paramIndex == PARAMS::P_SUPPORT_FOOT) {

      std::cout << "SOMETHING IS WRONG, This method should not call head or "
                   "support foot!!!"
                << paramIndex << std::endl; // "\r";
      return;
    }
    paramNameT paramIdxCopy =
        static_cast<PARAMS>(paramIndex - PARAMS::P_TORSO_POS_X);

    // first array index
    uint8_t initialBlock = paramIdxCopy / 3;
    // second array index
    uint8_t subIdx = paramIdxCopy % 3;

    minLim = min[initialBlock][subIdx];
    maxLim = max[initialBlock][subIdx];
    increment = inc[initialBlock][subIdx];
  }

  static MinMaxInc populateMinMaxInc(const Uni::Value &val) {
    MinMaxInc minMaxIncObj;

    val["min_headYawPitch"] >> minMaxIncObj.minHeadYawPitch;
    val["min_torsoPos"] >> minMaxIncObj.min[0];
    val["min_torsoRot"] >> minMaxIncObj.min[1];
    val["min_oFootPos"] >> minMaxIncObj.min[2];
    val["min_oFootRot"] >> minMaxIncObj.min[3];

    val["max_headYawPitch"] >> minMaxIncObj.maxHeadYawPitch;
    val["max_torsoPos"] >> minMaxIncObj.max[0];
    val["max_torsoRot"] >> minMaxIncObj.max[1];
    val["max_oFootPos"] >> minMaxIncObj.max[2];
    val["max_oFootRot"] >> minMaxIncObj.max[3];

    val["inc_headYawPitch"] >> minMaxIncObj.incHeadYawPitch;
    val["inc_torsoPos"] >> minMaxIncObj.inc[0];
    val["inc_torsoRot"] >> minMaxIncObj.inc[1];
    val["inc_oFootPos"] >> minMaxIncObj.inc[2];
    val["inc_oFootRot"] >> minMaxIncObj.inc[3];

    return minMaxIncObj;
  }
};

bool poseSensitivityStreamingTest() {
  std::vector<std::string> pOldLst = {"SENSV2 657042008010048220 1 3 8 3833 "
                                      "-7828 -24 9 -846 1313 -143 11 1346 -35 "
                                      "300 12 1341 -55 302 13 1347 -69 302 ",
                                      "SENSV2 657042008010048220 0 3 "};

  // No trailing space at end of entry.
  std::vector<std::string> pNewLst = {"SENSV2 657042008010048220 1 3 8 3833 "
                                      "-7828 -24 9 -846 1313 -143 11 1346 -35 "
                                      "300 12 1341 -55 302 13 1347 -69 302",
                                      "SENSV2 657042008010048220 0 3"};
  bool finalSuccess = true;
  /// Test if old space-trailing format can be read
  for (size_t i = 0; i < pOldLst.size(); i++) {
    PoseSensitivity<Vector3f> sens;
    std::stringstream sso(pOldLst[i]);
    std::stringstream ssi;

    sso >> sens;
    ssi << sens;
    // Old string must convert to new strings.
    bool success1 = pNewLst[i].compare(ssi.str()) == 0;
    if (!success1) {
      std::cout << ssi.str() << "|" << std::endl;
      std::cout << sso.str() << "|" << std::endl;
    }
    std::cout << "PoseSensitivity Istreamed vs Ostreamed compare [old -> new] "
              << std::to_string(i) << " : " << (success1 ? "success" : "fail")
              << std::endl;
    finalSuccess &= success1;
  }
  /// Test if new format can be read
  for (size_t i = 0; i < pNewLst.size(); i++) {
    PoseSensitivity<Vector3f> sens;
    std::stringstream sso(pNewLst[i]);
    std::stringstream ssi;

    sso >> sens;
    ssi << sens;
    // Old string must convert to new strings.
    bool success1 = pNewLst[i].compare(ssi.str()) == 0;
    if (!success1) {
      std::cout << ssi.str() << "|" << std::endl;
      std::cout << sso.str() << "|" << std::endl;
    }
    std::cout << "PoseSensitivity Istreamed vs Ostreamed compare [new -> new] "
              << std::to_string(i) << " : " << (success1 ? "success" : "fail")
              << std::endl;
    finalSuccess &= success1;
  }
  return finalSuccess;
}
bool poseStreamingTest() {
  NaoPoseAndRawAngles<float> poseAndAngles1;
  std::string p =
      "NaoPoseAndRawAngles NaoPoseV2 123456 0 -72 10 0 -0.1 -0.1 0.225 6 -12 0 "
      "-0.04 -0.1 0 0 0 0 -1.25664 0 "
      "1.5708 0.2 1.5708 -0.00872665 0 0 0.0895838 0.274045 -1.33344 1.64128 "
      "-0.157236 -0.384673 0.0895838 0.277598 "
      "-1.30291 2.00971 -0.543341 -0.365134 1.5708 -0.2 -1.5708 0.00872665 0 0";
  std::stringstream ss(p);
  ss >> poseAndAngles1;
  // std::cout << "PoseandAngles Istream" << std::endl;
  // std::cout << poseAndAngles1 << std::endl;
  // std::cout << std::endl;
  // std::cout << "PoseandAngles Ostream" << std::endl;
  // NaoPoseAndRawAngles<float> poseAndAngles(pose, joints);
  // std::cout << poseAndAngles1 << std::endl;
  // std::cout << std::endl;
  std::stringstream ss1;
  ss1 << poseAndAngles1;
  bool success = p.compare(ss1.str()) == 0;
  std::cout << "NaoPoseAndRaw Istreamed vs Ostreamed compare: "
            << (success ? "success" : "fail") << std::endl;

  std::cout << "orig \n"
            << poseAndAngles1.pose << "\n"
            << poseAndAngles1 << std::endl;

  poseAndAngles1.mirror();
  std::cout << "mirrored\n"
            << poseAndAngles1.pose << "\n"
            << poseAndAngles1 << std::endl;
  return success;
}

bool camObserverIterTest(const ObservationModelConfig cfg,
                         std::vector<float> joints, SUPPORT_FOOT supFoot) {

  ObservationSensitivity obs =
      ObservationSensitivityProvider::getSensitivityProvider(cfg);

  std::vector<PoseSensitivity<Vector3f>> sensitivityOutput =
      obs.getSensitivities(joints, supFoot);

  for (auto &i : sensitivityOutput) {
    Vector3f val;
    bool obs;
    for (int j = 0; j < JOINTS::JOINT::JOINTS_MAX; j++) {
      i.getSensitivity(static_cast<JOINTS::JOINT>(j), val, obs);
      // std::cout << "SENS-> " << j << " (" << val.x() << ", " << val.y() << ",
      // " << val.z() << ") o:" << obs << std::endl;
      if (obs) {
        std::cout << "SENS-> " << j << " " << val.norm() << " o:" << obs
                  << std::endl;
      }
    }
  }
  return true;
}

int main(int argc, char **argv) {
  std::string inFileName((argc > 1 ? argv[1] : "out"));
  std::string supportFootName((argc > 2 ? argv[2] : "d"));
  std::string confRoot((argc > 3 ? argv[3] : "../../nao/home/"));

  SUPPORT_FOOT supportFoot = SUPPORT_FOOT::SF_DOUBLE;
  if (supportFootName.compare("l") == 0) {
    supportFoot = SUPPORT_FOOT::SF_LEFT;
    std::cout << "Left Support foot." << std::endl;
  } else if (supportFootName.compare("r") == 0) {
    supportFoot = SUPPORT_FOOT::SF_RIGHT;
    std::cout << "Right Support foot." << std::endl;
  } else {
    std::cout << "Going for double foot." << std::endl;
    supportFootName = "d";
  }

  TUHH tuhhInstance(confRoot);

  Vector2f fc, cc, fov;

  tuhhInstance.config_.mount("Projection", "Projection.json",
                             ConfigurationType::HEAD);
  tuhhInstance.config_.get("Projection", "top_fc") >> fc;
  tuhhInstance.config_.get("Projection", "top_cc") >> cc;
  tuhhInstance.config_.get("Projection", "fov") >> fov;

  /**
   * MinMaxIncTest
   */
  Uni::Value confValue;
  if (!MiniConfigHandle::mountFile(
          "configuration/limits_" + supportFootName + ".json", confValue)) {
    std::cout << "couldn't open the conf file" << std::endl;
    exit(1);
  }
  const MinMaxInc minMaxInobj = MinMaxInc::populateMinMaxInc(confValue);

  std::cout << "minMaxIncTest" << std::endl;
  for (const auto &i : minMaxInobj.min) {
    std::cout << i.transpose() << " * ";
  }
  std::cout << std::endl;
  for (const auto &i : minMaxInobj.max) {
    std::cout << i.transpose() << " * ";
  }
  std::cout << std::endl;
  for (const auto &i : minMaxInobj.inc) {
    std::cout << i.transpose() << " * ";
  }
  std::cout << std::endl;

  for (int i = static_cast<PARAMS>(PARAMS::P_TORSO_POS_X);
       i < static_cast<PARAMS>(PARAMS::P_MAX); i++) {
    float min, max, inc;
    minMaxInobj.getLimits(static_cast<PARAMS>(i), min, max, inc);
    std::cout << i << " " << min << " " << max << " " << inc << std::endl;
  }
  /**
   * End MinMaxIncTest
   */

  Vector2i imSize(640, 480);

  CameraMatrix camMat;
  camMat.fc = fc;
  camMat.fc.x() *= imSize.x();
  camMat.fc.y() *= imSize.y();
  camMat.cc = cc;
  camMat.cc.x() *= imSize.x();
  camMat.cc.y() *= imSize.y();
  camMat.fov = fov;

  Vector2f point(0.4, 0.1);
  Vector2i pixPoint(0, 0);

  const ObservationModelConfig cfg = {
      imSize, fc, cc, fov, {SENSOR_NAME::BOTTOM_CAMERA}, 1000, 5, 0.05};

  std::cout << camMat.robotToPixel(point, pixPoint) << " " << pixPoint
            << std::endl;

  /**
   * Pose types streaming test
   */
  poseStreamingTest();

  /**
   * Pose sensitivity streaming test
   */
  poseSensitivityStreamingTest();

  /**
   * Observer test
   */
  std::vector<float> jointAngles(JOINTS::JOINT::JOINTS_MAX, 0.0);
  jointAngles[JOINTS::JOINT::HEAD_PITCH] = 20 * TO_RAD;
  std::cout << "start camobserver test" << std::endl;
  camObserverIterTest(cfg, std::vector<float>(jointAngles), supportFoot);

  return 0;
}
