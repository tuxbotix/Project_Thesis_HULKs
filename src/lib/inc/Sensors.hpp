#pragma once

#include <Modules/NaoProvider.h>
#include <vector>

enum SENSOR_NAME {
  SENSOR_NONE = -1,
  TOP_CAMERA = 0,
  BOTTOM_CAMERA = 1
  // Future : 3d measurements, etc on lifted foot, etc.
};

class Sensor {
public:
  SENSOR_NAME name;

  static const std::vector<JOINTS::JOINT> CAM_OBS_L_SUP_FOOT;
  static const std::vector<JOINTS::JOINT> CAM_OBS_R_SUP_FOOT;

  static const std::vector<JOINTS::JOINT>
      ALL_OBS_JOINTS; // all observed joints, excludes arms..
  static const std::vector<std::string>
      JOINT_NAMES; // all observed joints, excludes arms..
  static void init() {}
};
