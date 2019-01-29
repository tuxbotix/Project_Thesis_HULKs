#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
// #include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <thread>
#include <tuple>
#include <unordered_set>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Tools/Math/Eigen.hpp>
#include <unsupported/Eigen/NonLinearOptimization>

#include <CppNumericalSolvers/include/cppoptlib/problem.h>
#include <CppNumericalSolvers/include/cppoptlib/solver/cmaessolver.h>

#include "NaoJointAndSensorModel.hpp"
#include "NaoPoseInfo.hpp"
#include "NaoStability.hpp"
#include "NaoTorsoPose.hpp"
#include "constants.hpp"

namespace JointCalibSolvers {

typedef std::vector<std::pair<int, std::vector<float>>> JointAndPoints;
// first is ground point
typedef std::vector<std::pair<Vector2f, Vector2f>> CorrespondanceList;
typedef std::vector<std::pair<Vector3f, Vector2f>> Correspondance3D2DList;

const long LEG_JOINT_COUNT =
    JOINTS::JOINT::L_ANKLE_ROLL - JOINTS::JOINT::L_HAND;
const long COMPACT_JOINT_CALIB_PARAM_COUNT =
    2 + LEG_JOINT_COUNT * 2; // head->2 + legs = 6*2

// compact calib packing format: headY, headP, LLegjoints, RLegJoints
/**
 * @brief calibParamsToRawPose
 * @param params
 * @param vec
 */
template <typename FromType, typename ToType = FromType>
inline bool jointCalibParamsToRawPose(const VectorX<FromType> &params,
                                      std::vector<ToType> &vec) {
  if (params.size() < COMPACT_JOINT_CALIB_PARAM_COUNT) {
    return false;
  }
  if (vec.size() < JOINTS::JOINT::JOINTS_MAX) {
    vec.resize(JOINTS::JOINT::JOINTS_MAX);
  }
  vec[JOINTS::JOINT::HEAD_PITCH] =
      static_cast<ToType>(params(JOINTS::JOINT::HEAD_PITCH));
  vec[JOINTS::JOINT::HEAD_YAW] =
      static_cast<ToType>(params(JOINTS::JOINT::HEAD_YAW));

  // left
  for (Eigen::Index i = 0; i < LEG_JOINT_COUNT; i++) {
    vec[static_cast<size_t>(JOINTS::JOINT::L_HIP_YAW_PITCH + i)] =
        static_cast<ToType>(params(2 + i));
    vec[static_cast<size_t>(JOINTS::JOINT::R_HIP_YAW_PITCH + i)] =
        static_cast<ToType>(params(2 + LEG_JOINT_COUNT + i));
  }
  return true;
}

template <typename FromType, typename ToType = FromType>
inline bool rawPoseToJointCalibParams(const std::vector<FromType> &vec,
                                      VectorX<ToType> &params) {
  if (vec.size() < JOINTS::JOINT::JOINTS_MAX) {
    return false;
  }
  if (params.size() < COMPACT_JOINT_CALIB_PARAM_COUNT) {
    params.resize(COMPACT_JOINT_CALIB_PARAM_COUNT);
  }
  params(JOINTS::JOINT::HEAD_PITCH) =
      static_cast<ToType>(vec[JOINTS::JOINT::HEAD_PITCH]);
  params(JOINTS::JOINT::HEAD_YAW) =
      static_cast<ToType>(vec[JOINTS::JOINT::HEAD_YAW]);

  for (Eigen::Index i = 0; i < LEG_JOINT_COUNT; i++) {
    params(2 + i) =
        static_cast<ToType>(vec[JOINTS::JOINT::L_HIP_YAW_PITCH + i]);
    params(2 + LEG_JOINT_COUNT + i) =
        static_cast<ToType>(vec[JOINTS::JOINT::R_HIP_YAW_PITCH + i]);
  }
  return true;
}

/**
 * @brief The JointCalibCaptureEvalT struct
 */
struct JointCalibCaptureEvalT {
  CorrespondanceList capturedCorrespondances;
  rawPoseT pose;
  SUPPORT_FOOT sf;
  Camera camName;

  JointCalibCaptureEvalT(const CorrespondanceList correspondances,
                         const rawPoseT &pose, const Camera camName,
                         const SUPPORT_FOOT sf)
      : capturedCorrespondances(correspondances), pose(pose), sf(sf),
        camName(camName) {}
};

/**
 * @brief The JointCalibResult struct
 */
struct JointCalibResult {
  int status; // out of solver
  float reprojectionErrorNorm;
  float reprojectionErrorAvg;
  rawPoseT jointParams; // solved params
};

typedef std::vector<JointCalibCaptureEvalT> CaptureList;

/*
 * Below is Eigen based non-global optimization
 */

/*
 * Extracted from
 * https://github.com/daviddoria/Examples/blob/master/c%2B%2B/Eigen/LevenbergMarquardt/CurveFitting.cpp
 * This definition structure is needed to use NumericalDiff module
 */

template <typename _Scalar = float, int NX = Eigen::Dynamic,
          int NY = Eigen::Dynamic>
/**
 * @brief The Functor struct
 */
struct Functor {
  typedef _Scalar Scalar;
  enum { InputsAtCompileTime = NX, ValuesAtCompileTime = NY };
  typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
  typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
  typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime>
      JacobianType;

  size_t n_inputs, m_values;

  Functor() : n_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
  Functor(size_t inputs, size_t values) : n_inputs(inputs), m_values(values) {}

  size_t inputs() const { return n_inputs; }
  size_t values() const { return m_values; }
};

/**
 * @brief The JointCalibrator struct
 */
struct JointCalibrator : Functor<float> {

  const CaptureList captureList;
  const size_t captureDataSetSize;
  const NaoJointAndSensorModelConfig cfg;

  /**
   * @brief operator () Almost the flavour liked by Eigen's solvers
   * @param calibrationValsStdVec testParameters, as std::vector, FULL LENGTH***
   * @param errorVec residual vector
   * @return state of the function.
   */
  int operator()(const std::vector<float> &calibrationValsStdVec,
                 ValueType &errorVec) const {

    Eigen::Index iter = 0;
    for (const auto &capture : captureList) {
      //            auto & errorAtCap = errorVecVec[capIdx];
      //            errorAtCap.reserve(capture.capturedCorrespondances.size());
      const auto &camName = capture.camName;

      auto naoJointSensorModel = NaoJointAndSensorModel(cfg);
      naoJointSensorModel.setCalibValues(calibrationValsStdVec);
      naoJointSensorModel.setPose(capture.pose, capture.sf);
      //      Vector2d accum;
      for (const auto &correspondance : capture.capturedCorrespondances) {
        Vector2f error(0, 0);

        naoJointSensorModel.robotToPixelFlt(camName, correspondance.first,
                                            error);
        error = correspondance.second - error;
        //        errorVec(iter) = static_cast<Scalar>(error.norm());
        errorVec(iter) = static_cast<Scalar>(error.x());
        ++iter;
        errorVec(iter) = static_cast<Scalar>(error.y());
        ++iter;
      }
    }
    return 0;
  }

  /**
   * @brief operator () Flavour liked by Eigen's solvers (it needs
   * Eigen::VectorXx)
   * This just maps the eigen::vector to std::vector and call above func.
   * @param calibrationVals same as above function, but Eig::VectorXf AND
   * SHORTENED, only head and leg angles!!!
   * @param errorVec residual vector
   * @return return of above func
   */
  int operator()(const InputType &calibrationVals, ValueType &errorVec) const {
    rawPoseT calibrationValsStdVec(JOINTS::JOINT::JOINTS_MAX);
    if (!jointCalibParamsToRawPose<Scalar, float>(calibrationVals,
                                                  calibrationValsStdVec)) {
      std::cerr << "Yelp" << std::endl;
    }
    return this->operator()(calibrationValsStdVec, errorVec);
  }

  /**
   * @brief JointCalibrator The constructor!
   * @param captures captures - 3d-2d correspondances
   * @param model naoJoint model
   */
  JointCalibrator(CaptureList &captures, NaoJointAndSensorModelConfig cfg)
      : Functor(COMPACT_JOINT_CALIB_PARAM_COUNT,
                [=]() -> size_t {
                  size_t temp = 0;
                  for (const auto &cap : captures) {
                    temp += cap.capturedCorrespondances.size() * 2;
                  }
                  return temp;
                }()),
        captureList(captures),
        // This nasty chunk get total element count via a lambda and return to
        // the int.
        captureDataSetSize(Functor::values()), cfg(cfg) {}

  // Means the parameter count = 26~ in our case
  int inputs() const { return COMPACT_JOINT_CALIB_PARAM_COUNT; }
  // size of sample/ correspondance set
  size_t values() const { return captureDataSetSize; }
};

} // namespace JointCalibSolvers
