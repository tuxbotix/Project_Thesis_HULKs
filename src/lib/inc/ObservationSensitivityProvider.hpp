#pragma once

#include <cmath>
#include <memory>

#include "Data/CameraMatrix.hpp"

#include "NaoPoseInfo.hpp"
#include "NaoProjectionDataProvider.hpp"

#include "CameraObservationModel.hpp"
#include "ObservationModel.hpp"

/**
 * To be precise, this give sensitivity of camera observation :P
 */
class ObservationSensitivity {
  // public:
private:
  // TODO make this public later
  // class CameraObservationModel;
  std::shared_ptr<CameraObservationModel> camObsModelPtr;
  const std::vector<SENSOR_NAME> sensorNames;

private:
  friend class ObservationSensitivityProvider;
  ObservationSensitivity(const Vector2i &imSize, const Vector2f &fc,
                         const Vector2f &cc, const Vector2f &fov,
                         const std::vector<SENSOR_NAME> &sensors,
                         uint64_t dimensionExtremum,
                         const size_t &maxGridPointsPerSide,
                         const float &gridSpacing)
      : camObsModelPtr(std::make_shared<CameraObservationModel>(
            imSize, fc, cc, fov, dimensionExtremum, maxGridPointsPerSide,
            gridSpacing)),
        sensorNames(sensors) {
  }

  /**
   * Get sensitivities for each joint and each sensor
   *
   * TODO make this flexible.
   */
public:
  std::vector<PoseSensitivity<Vector3f>>
  getSensitivities(const rawPoseT &jointAngles, const SUPPORT_FOOT &sf) {
    std::vector<PoseSensitivity<Vector3f>> output;
    for (const auto &i : sensorNames) {
      if (i == SENSOR_NAME::TOP_CAMERA || i == SENSOR_NAME::BOTTOM_CAMERA) {
        output.push_back(
            camObsModelPtr->getSensitivitiesForCamera(jointAngles, sf, i));
      } else {
        std::cerr << "Sensor Sensitivity Function not implemented."
                  << std::endl;
        throw "Sensor Sensitivity Function not implemented.";
      }
    }
    return output;
  }
};

struct ObservationModelConfig {
  Vector2i imSize;
  Vector2f fc;
  Vector2f cc;
  Vector2f fov;
  std::vector<SENSOR_NAME> sensorNames;
  uint64_t dimensionExtremum;
  size_t maxGridPointsPerSide;
  float gridSpacing;
};

class ObservationSensitivityProvider {
public:
  /**
     * ctor
     * @param threadCount allows automatic initializing of enough camera
   * matrices..
     */

  static std::vector<ObservationSensitivity>
  getSensitivityProviders(const size_t threadCount,
                          const ObservationModelConfig cfg) {

    return std::vector<ObservationSensitivity>(
        threadCount,
        ObservationSensitivity(cfg.imSize, cfg.fc, cfg.cc, cfg.fov,
                               cfg.sensorNames, cfg.dimensionExtremum,
                               cfg.maxGridPointsPerSide, cfg.gridSpacing));
  }

  static ObservationSensitivity
  getSensitivityProvider(const ObservationModelConfig cfg) {
    return ObservationSensitivity(cfg.imSize, cfg.fc, cfg.cc, cfg.fov,
                                  cfg.sensorNames, cfg.dimensionExtremum,
                                  cfg.maxGridPointsPerSide, cfg.gridSpacing);
  }
};

/**
 * This class is for simplifying testing purposes.
 */

// class ObservationModelFactory
// {
// private:
//   ObservationModelFactory() {}
//   ~ObservationModelFactory() {}

// public:
//   static ObservationModel getObservationModel(std::string obsName)
//   {
//     switch(obsName){
//       case "CameraObservationModel":
//       return CameraObservationModel()
//     }
//   }
// };
