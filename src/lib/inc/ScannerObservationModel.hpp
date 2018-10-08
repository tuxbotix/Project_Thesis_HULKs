#pragma once

#include "ObservationModel.hpp"
/**
 * This class is to model the observation of joint errors [ie: small variation of a joint value].
 * It's for the case which the Nao's foot is lifted and it's foot is looked at with a 3D scanner.
 * The 3D scanner shall get Z height and Roll, Pitch angles.
 * X, Y translation or Yaw angle is tricky as getting these will be relative to support foot.
 * Therefore, for simplicity sake, we'll only consider 3 dimensions.
 * 
 * TODO Determine "bounds" for Z height offset and the Roll Pitch angles.
 */

class ScannerObservationModel : public ObservationModel
{
  public:
    ~ScannerObservationModel()
    {
    }
    ScannerObservationModel() {}
    std::string getName()
    {
        return "ScannerObservationModel";
    }
};