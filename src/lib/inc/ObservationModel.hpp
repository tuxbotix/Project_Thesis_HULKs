#pragma once

#include "NaoPoseInfo.hpp"
// #include "ObservationSensitivityProvider.hpp"
/**
 * This is the base class for observation models.
 * Currently I'll leave this as an abstract class
 */

class ObservationModel {
public:
  // friend class ObservationSensitivity;

  //   private:
  virtual inline ~ObservationModel() {}
  virtual std::string getName() = 0;
  // template <typeName T> // Must be a VectorXf type.
  // virtual T getSensitivities() = 0;
};
