#include <fstream>
#include <iostream>
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

#include "CalibrationFeatures.hpp"

int main() {
  auto pattern = CalibrationFeatures::ChessBoardPatternOnFloor<float>(
      10, 5, 1, Vector2f(10, 5), 90 * TO_RAD_FLT);
  auto points = pattern.getGroundPoints();
  for (const auto &pt : points) {
    std::cout << pt.transpose() << std::endl;
  }
}
