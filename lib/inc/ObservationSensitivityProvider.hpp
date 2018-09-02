#pragma once

#include <cmath>
#include <memory>

#include "Data/CameraMatrix.hpp"

#include "NaoPoseInfo.hpp"
#include "NaoProjectionDataProvider.hpp"

#include "ObservationModel.hpp"
#include "CameraObservationModel.hpp"

/**
 * To be precise, this give sensitivity of camera observation :P
 */
class ObservationSensitivity
{
// public:
private:
  // TODO make this public later
  // class CameraObservationModel;
  std::shared_ptr<CameraObservationModel> camObsModelPtr;

private:
  friend class ObservationSensitivityProvider;
  ObservationSensitivity(const Vector2i &imSize, const Vector2f &fc, const Vector2f &cc, const Vector2f &fov,
                         uint64_t dimensionExtremum, const size_t &maxGridPointsPerSide, const float &gridSpacing)
      : camObsModelPtr(std::make_shared<CameraObservationModel>(imSize, fc, cc, fov, dimensionExtremum, maxGridPointsPerSide, gridSpacing))
  {
  }

  /**
   * Get sensitivities for each joint and each sensor
   * 
   * TODO make this flexible.
   */
public:
  std::vector<PoseSensitivity<Vector3f>> getSensitivities(const rawPoseT &jointAngles, const SUPPORT_FOOT &sf, const std::vector<SENSOR_NAME> &sensorNames)
  {
    std::vector<PoseSensitivity<Vector3f>> output;
    for (const auto &i : sensorNames)
    {
      if (i == SENSOR_NAME::TOP_CAMERA || i == SENSOR_NAME::BOTTOM_CAMERA)
      {
        output.push_back(camObsModelPtr->getSensitivitiesForCamera(jointAngles, sf, i));
      }
      else
      {
        throw "Sensor Sensitivity Function not implemented.";
      }
    }
    return output;
  }
};

class ObservationSensitivityProvider
{
public:
  /**
     * ctor
     * @param threadCount allows automatic initializing of enough camera matrices..
     */

  static std::vector<ObservationSensitivity> getSensitivityProviders(const size_t &threadCount, const Vector2i &imSize,
                                                                     const Vector2f &fc, const Vector2f &cc,
                                                                     const Vector2f &fov,
                                                                     const uint64_t dimensionExtremum,
                                                                     const size_t &maxGridPointsPerSide, const float &gridSpacing)
  {

    return std::vector<ObservationSensitivity>(threadCount, ObservationSensitivity(imSize, fc, cc, fov, dimensionExtremum, maxGridPointsPerSide, gridSpacing));
  }

  static ObservationSensitivity getSensitivityProvider(const Vector2i imSize,
                                                       const Vector2f fc, const Vector2f cc,
                                                       const Vector2f fov,
                                                       const uint64_t dimensionExtremum,
                                                       const size_t maxGridPointsPerSide, const float gridSpacing)
  {
    return ObservationSensitivity(imSize, fc, cc, fov, dimensionExtremum, maxGridPointsPerSide, gridSpacing);
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