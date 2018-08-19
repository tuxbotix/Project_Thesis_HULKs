#pragma once

#include "Data/CameraMatrix.hpp"

#include "NaoPoseInfo.hpp"
#include "NaoProjectionDataProvider.hpp"

#include <unsupported/Eigen/NonLinearOptimization>
/**
 * To be precise, this give sensitivity of camera observation :P
 */
class ObservationSensitivity
{
private:
  float gridSpacing;
  const float deltaThetaCorse;
  const float deltaThetaFine;

  // When divided by this, all outputs must scale between -1 and +1
  const Vector3f dimensionScale;
  // const Vector3f dimensionMax;
  CameraMatrix camMat;

  friend class ObservationSensitivityProvider;
  ObservationSensitivity(const Vector2i &imSize, const Vector2f &fc, const Vector2f &cc, const Vector2f &fov,
                         const float &gridSpacing)
      : gridSpacing(gridSpacing), deltaThetaCorse(5.0f), deltaThetaFine(1.0f), dimensionScale(imSize.x() / 2, imSize.y() / 2, M_PI_2f64)
  {
    camMat.fc = fc;
    camMat.fc.x() *= imSize.x();
    camMat.fc.y() *= imSize.y();
    camMat.cc = cc;
    camMat.cc.x() *= imSize.x();
    camMat.cc.y() *= imSize.y();
    camMat.fov = fov;
  }

public:
  // std::vector<Vector3f> getSensitivities(rawPoseT & jointAngles, SUPPORT_FOOT)
  // {
  // }
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
                                                                     const Vector2f &fov, const float &gridSpacing)
  {
    return std::vector<ObservationSensitivity>(threadCount, ObservationSensitivity(imSize, fc, cc, fov, gridSpacing));
  }
};

namespace ObservationQuality
{

/**
 * Below modified implementation extracted from
 * https://github.com/SarvagyaVaish/Eigen-Levenberg-Marquardt-Optimization
 */
struct Pose2DFunctor
{
  // 'm' pairs of (x, f(x))
  Eigen::MatrixXf measuredValues;
  Eigen::MatrixXf initialValues;

  void populateDataMatrices(const std::vector<float> &initialPoints, const std::vector<float> &measuements)
  {
    // TODO chweck if the input vectors have same size
    m = initialPoints.size() / 2;
    measuredValues = Eigen::MatrixXf(m, 2);
    initialValues = Eigen::MatrixXf(m, 2);
    for (size_t i = 0; i < m; i++)
    {
      measuredValues(i, 0) = measuements[i * 2];
      measuredValues(i, 1) = measuements[(i * 2) + 1];
      initialValues(i, 0) = initialPoints[i * 2];
      initialValues(i, 1) = initialPoints[(i * 2) + 1];
    }
  }

  // Compute 'm' errors, one for each data point, for the given parameter values in 'x'
  int operator()(const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const
  {
    // 'x' has dimensions n x 1
    // It contains the current estimates for the parameters.

    // 'fvec' has dimensions m x 1
    // It will contain the error for each data point.

    float aParam = x(0);
    float bParam = x(1);
    float cParam = x(2);
    Eigen::Translation<float, 2> trans(aParam, bParam);
    Eigen::Rotation2D<float> rot2(cParam * TO_RAD);

    for (size_t i = 0; i < m; i++)
    {
      Eigen::Vector2f val = trans * rot2 * Eigen::Vector2f(initialValues.row(i));

      fvec(i) = (val - Eigen::Vector2f(measuredValues.row(i))).norm();
    }
    return 0;
  }

  // Compute the jacobian of the errors
  int df(const Eigen::VectorXf &x, Eigen::MatrixXf &fjac) const
  {
    // 'x' has dimensions n x 1
    // It contains the current estimates for the parameters.

    // 'fjac' has dimensions m x n
    // It will contain the jacobian of the errors, calculated numerically in this case.

    float epsilon;
    epsilon = 1e-5f;

    for (int i = 0; i < x.size(); i++)
    {
      Eigen::VectorXf xPlus(x);
      xPlus(i) += epsilon;
      Eigen::VectorXf xMinus(x);
      xMinus(i) -= epsilon;

      Eigen::VectorXf fvecPlus(m);
      operator()(xPlus, fvecPlus);

      Eigen::VectorXf fvecMinus(m);
      operator()(xMinus, fvecMinus);

      Eigen::VectorXf fvecDiff(m);
      fvecDiff = (fvecPlus - fvecMinus) / (2.0f * epsilon);

      fjac.block(0, i, m, 1) = fvecDiff;
    }

    return 0;
  }

  // Number of data points, i.e. values.
  size_t m;

  // Returns 'm', the number of values.
  size_t values() const { return m; }

  // The number of parameters, i.e. inputs.
  size_t n;

  // Returns 'n', the number of inputs.
  size_t inputs() const { return n; }
};

/**
 * This is quick n dirty way to call this..
 */

void get2dPose(const std::vector<float> &initialPoints,
               const std::vector<float> &transformedPoints, Eigen::Vector3f &params)
{
  if (initialPoints.size() != transformedPoints.size())
  {
    std::cerr << "input list dimension mismatch" << std::endl;
    return;
  }
  VectorXf x(params);

  Pose2DFunctor functor;
  functor.populateDataMatrices(initialPoints, transformedPoints);
  functor.n = 3;

  Eigen::LevenbergMarquardt<Pose2DFunctor, float> lm(functor);
  int status = lm.minimize(x);
  params = Eigen::Vector3f(x);
  std::cout << "LM optimization status: " << status << std::endl;
}

} // namespace ObservationQuality