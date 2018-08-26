#pragma once

#include <vector>
#include <Modules/NaoProvider.h>

enum SENSOR_NAME
{
  TOP_CAMERA,
  BOTTOM_CAMERA
  // Future : 3d measurements, etc on lifted foot, etc.
};

class Sensor
{
public:
  SENSOR_NAME name;

  static const std::vector<JOINTS::JOINT> CAM_OBS_L_SUP_FOOT;
  static const std::vector<JOINTS::JOINT> CAM_OBS_R_SUP_FOOT;

  static void init(){
    
  }
};