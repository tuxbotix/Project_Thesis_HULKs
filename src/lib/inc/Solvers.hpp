#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <unsupported/Eigen/NonLinearOptimization>

//#include "cppoptlib/boundedproblem.h"
//#include "cppoptlib/meta.h"
//#include "cppoptlib/solver/lbfgsbsolver.h"

namespace ObservationSolvers {

typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>
    Point2DVector;
typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>
    Point2FVector;

/**
 * Extracted from
 * https://github.com/daviddoria/Examples/blob/master/c%2B%2B/Eigen/LevenbergMarquardt/CurveFitting.cpp
 * This definition structure is needed to use NumericalDiff module
 */

template <typename _Scalar = float, int NX = Eigen::Dynamic,
          int NY = Eigen::Dynamic>
struct Functor {
  typedef _Scalar Scalar;
  enum { InputsAtCompileTime = NX, ValuesAtCompileTime = NY };
  typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
  typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
  typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime>
      JacobianType;

  int n_inputs, m_values;

  Functor() : n_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
  Functor(int inputs, int values) : n_inputs(inputs), m_values(values) {}

  int inputs() const { return n_inputs; }
  int values() const { return m_values; }
};

template <typename _Scalar = float> struct Pose2DFunctor : Functor<_Scalar> {
  // 'm' pairs of (x, f(x))
  std::vector<_Scalar> initialPoints;
  std::vector<_Scalar> transformedPoints;

  // Compute 'm' errors, one for each data point, for the given parameter values
  // in 'x'
  int operator()(const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const {

    _Scalar xTrans = x(0);
    _Scalar yTrans = x(1);
    _Scalar zRot = x(2);
    Eigen::Translation<_Scalar, 2> trans(xTrans, yTrans);
    Eigen::Rotation2D<_Scalar> rot2(zRot * TO_RAD);
    Eigen::UniformScaling<float> scale(x(3));
    Eigen::Vector2f diff;
    Eigen::Vector2f tempVec;

    for (size_t i = 0; i < initialPoints.size(); i += 2) {
      // diff = transformedPoints[i] - (trans * rot2 * initialPoints[i]);
      tempVec.x() = initialPoints[i];
      tempVec.y() = initialPoints[i + 1];
      diff = Vector2f(transformedPoints[i], transformedPoints[i + 1]) -
             (trans * scale * rot2 * tempVec);
      // fvec(i/2) = diff.dot(diff);
      fvec(i) = diff.x();
      fvec(i + 1) = diff.y();
    }
    return 0;
  }

  int inputs() const { return 3; }
  int values() const { return initialPoints.size(); }
};

/**
 * CppOptLib problem
 */

// we will solve ||Xb-y|| s.t. b>=0
// template <typename T>
// class Pose2DLeastSquares : public cppoptlib::BoundedProblem<T> {
// public:
//  using Superclass = cppoptlib::BoundedProblem<T>;
//  using typename Superclass::TVector;
//  using TMatrix = typename Superclass::THessian;

//  const TMatrix X;
//  const TVector y;

// public:
//  Pose2DLeastSquares(const TMatrix &X_, const TVector y_)
//      : Superclass(X_.cols()), X(X_), y(y_) {}

//  T value(const TVector &beta) { return (X * beta - y).dot(X * beta - y); }

//  void gradient(const TVector &beta, TVector &grad) {
//    grad = X.transpose() * 2 * (X * beta - y);
//  }
//};

/**
 * This is quick n dirty way to call this..
 */

// template <typename _Scalar>
int get2dPose(
    const std::vector<std::pair<Vector2f, Vector2f>> &correspondancePairs,
    Vector3f &params) {
  VectorXf paramsX(4); // 2 translations, 1 rotations and 2 scaling
  paramsX(0) = params.x();
  paramsX(1) = params.y();
  paramsX(2) = params.z();
  paramsX(3) = 1; // scale
  // paramsX(4) = 1; // scale

  std::vector<float> initialPoints;
  std::vector<float> transformedPoints;

  for (const auto &i : correspondancePairs) {
    initialPoints.push_back(i.first.x());
    initialPoints.push_back(i.first.y());
    transformedPoints.push_back(i.second.x());
    transformedPoints.push_back(i.second.y());
  }

  Pose2DFunctor<float> functor;
  functor.initialPoints = initialPoints;
  functor.transformedPoints = transformedPoints;

  { // Initial test, if the fuctor give almost no error, then just stop
    VectorXf diffs(functor.values());
    functor(paramsX, diffs);
    if (diffs.norm() < sqrt(Eigen::NumTraits<float>::epsilon())) {
      // std::cout << "skip" << std::endl;
      return 4;
    }
  }

  Eigen::NumericalDiff<Pose2DFunctor<float>> functorDiff(functor);
  Eigen::LevenbergMarquardt<Eigen::NumericalDiff<Pose2DFunctor<float>>, float>
      lm(functorDiff);

  int status = lm.minimize(paramsX);
  {
    VectorXf diffs(functor.values());
    functor(paramsX, diffs);
    const auto finalErrorAvg = diffs.mean();
    if (finalErrorAvg > 480 * 0.02) {
      status = -1;
    }
    // if (diffs.norm() > 5)
    //    {
    //      float mean = diffs.mean();
    //      VectorXf val = (diffs - VectorXf::Ones(diffs.size()) * mean);
    //      double std_dev = std::sqrt(val.dot(val) / (diffs.size() - 1));

    //      std::cout << "Solver's efficiency?? mean" << mean
    //                << " std dev: " << std_dev << " norm " << diffs.norm()
    //                << std::endl;
    //      // " min " << diffs.minCoeff() << " max " << diffs.maxCoeff() <<
    //      std::endl;
    //    }
  }
  params = Eigen::Vector3f(paramsX(0), paramsX(1), paramsX(2));
  return status;
}

} // namespace ObservationSolvers
