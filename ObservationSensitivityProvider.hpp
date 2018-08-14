#pragma once

#include "Data/CameraMatrix.hpp"

#include "NaoProjectionDataProvider.hpp"

/**
 * To be precise, this give sensitivity of camera observation :P
 */
class ObservationSensitivity
{
  private:
    CameraMatrix camMat;
    float gridSpacing;
    friend class ObservationSensitivityProvider;
    ObservationSensitivity(const Vector2i &imSize, const Vector2f &fc, const Vector2f &cc, const Vector2f &fov, const float &gridSpacing) : gridSpacing(gridSpacing)
    {
        camMat.fc = fc;
        camMat.fc.x() *= imSize.x();
        camMat.fc.y() *= imSize.y();
        camMat.cc = cc;
        camMat.cc.x() *= imSize.x();
        camMat.cc.y() *= imSize.y();
        camMat.fov = fov;
    }
};

class ObservationSensitivityProvider
{
  public:
    /**
     * ctor
     * @param threadCount allows automatic initializing of enough camera matrices..
     */

    static std::vector<ObservationSensitivity> getSensitivityProviders(const size_t &threadCount, const Vector2i &imSize, const Vector2f &fc, const Vector2f &cc,
                                                                       const Vector2f &fov, const float &gridSpacing)
    {
        return std::vector<ObservationSensitivity>(threadCount, ObservationSensitivity(imSize, fc, cc, fov, gridSpacing));
    }
};
