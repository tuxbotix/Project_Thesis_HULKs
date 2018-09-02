#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <unsupported/Eigen/NonLinearOptimization>

namespace ObservationSolvers
{

typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> Point2DVector;
typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>> Point2FVector;

/**
 * Extracted from https://github.com/daviddoria/Examples/blob/master/c%2B%2B/Eigen/LevenbergMarquardt/CurveFitting.cpp
 * This definition structure is needed to use NumericalDiff module
 */

template <typename _Scalar = float, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
struct Functor
{
  typedef _Scalar Scalar;
  enum
  {
    InputsAtCompileTime = NX,
    ValuesAtCompileTime = NY
  };
  typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
  typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
  typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

  int n_inputs, m_values;

  Functor() : n_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
  Functor(int inputs, int values) : n_inputs(inputs), m_values(values) {}

  int inputs() const { return n_inputs; }
  int values() const { return m_values; }
};

template <typename _Scalar = float>
struct Pose2DFunctor : Functor<_Scalar>
{
  // 'm' pairs of (x, f(x))
  Point2FVector initialPoints;
  Point2FVector transformedPoints;

  // Compute 'm' errors, one for each data point, for the given parameter values in 'x'
  int operator()(const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const
  {

    _Scalar xTrans = x(0);
    _Scalar yTrans = x(1);
    _Scalar zRot = x(2);
    Eigen::Translation<_Scalar, 2> trans(xTrans, yTrans);
    Eigen::Rotation2D<_Scalar> rot2(zRot * TO_RAD);
    Vector2f diff;

    for (size_t i = 0; i < initialPoints.size(); i++)
    {
      diff = transformedPoints[i] - (trans * rot2 * initialPoints[i]);
      fvec(i) = diff.norm();
    }
    return 0;
  }

  int inputs() const { return 3; }
  int values() const { return initialPoints.size(); }
};

/**
 * This is quick n dirty way to call this..
 */

// template <typename _Scalar>
int get2dPose(const std::vector<std::pair<Vector2<float>, Vector2<float>>> &correspondancePairs,
              Vector3<float> &params)
{
  VectorXf paramsX(3);
  paramsX(0) = params.x();
  paramsX(1) = params.y();
  paramsX(2) = params.z();

  Point2FVector initialPoints;
  Point2FVector transformedPoints;

  for (const auto &i : correspondancePairs)
  {
    initialPoints.push_back(i.first);
    transformedPoints.push_back(i.second);
  }

  Pose2DFunctor<float> functor;
  functor.initialPoints = initialPoints;
  functor.transformedPoints = transformedPoints;

  { // Initial test, if the fuctor give almost no error, then just stop
    VectorXf diffs(initialPoints.size());
    functor(paramsX, diffs);
    if (diffs.norm() < sqrt(Eigen::NumTraits<float>::epsilon()))
    {
      return 4;
    }
  }

  Eigen::NumericalDiff<Pose2DFunctor<float>> functorDiff(functor);
  Eigen::LevenbergMarquardt<Eigen::NumericalDiff<Pose2DFunctor<float>>, float> lm(functorDiff);

  int status = lm.minimize(paramsX);

  params = Eigen::Vector3f(paramsX);
  return status;
}

} // namespace ObservationSolvers
