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
#include <unsupported/Eigen/NonLinearOptimization>

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
inline bool jointCalibParamsToRawPose(const Eigen::VectorXf &params,
                                      std::vector<float> &vec) {
  if (params.size() < COMPACT_JOINT_CALIB_PARAM_COUNT) {
    return false;
  }
  if (vec.size() < JOINTS::JOINT::JOINTS_MAX) {
    vec.resize(JOINTS::JOINT::JOINTS_MAX);
  }
  vec[JOINTS::JOINT::HEAD_PITCH] = params(JOINTS::JOINT::HEAD_PITCH);
  vec[JOINTS::JOINT::HEAD_YAW] = params(JOINTS::JOINT::HEAD_YAW);

  // left
  for (long i = 0; i < LEG_JOINT_COUNT; i++) {
    vec[JOINTS::JOINT::L_HIP_YAW_PITCH + i] = params(2 + i);
    vec[JOINTS::JOINT::R_HIP_YAW_PITCH + i] = params(2 + LEG_JOINT_COUNT + i);
  }
  return true;
}

inline bool rawPoseToJointCalibParams(const std::vector<float> &vec,
                                      Eigen::VectorXf &params) {
  if (vec.size() < JOINTS::JOINT::JOINTS_MAX) {
    return false;
  }
  if (params.size() < COMPACT_JOINT_CALIB_PARAM_COUNT) {
    params.resize(COMPACT_JOINT_CALIB_PARAM_COUNT);
  }
  params(JOINTS::JOINT::HEAD_PITCH) = vec[JOINTS::JOINT::HEAD_PITCH];
  params(JOINTS::JOINT::HEAD_YAW) = vec[JOINTS::JOINT::HEAD_YAW];

  for (long i = 0; i < LEG_JOINT_COUNT; i++) {
    params(2 + i) = vec[JOINTS::JOINT::L_HIP_YAW_PITCH + i];
    params(2 + LEG_JOINT_COUNT + i) = vec[JOINTS::JOINT::R_HIP_YAW_PITCH + i];
  }
  return true;
}

// struct JointCalibCapture{
//    NaoPose<float> pose;
//    Correspondance3D2DList capturedCorrespondances;
//};

/**
 * @brief The JointCalibCaptureEvalT struct
 */
struct JointCalibCaptureEvalT {
  CorrespondanceList capturedCorrespondances;
  rawPoseT pose;
  SUPPORT_FOOT sf;
  Camera camName;

  JointCalibCaptureEvalT(const CorrespondanceList &correspondances,
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

  int n_inputs, m_values;

  Functor() : n_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
  Functor(int inputs, int values) : n_inputs(inputs), m_values(values) {}

  int inputs() const { return n_inputs; }
  int values() const { return m_values; }
};

/**
 * @brief The JointCalibrator struct
 */
struct JointCalibrator : Functor<float> {

  const CaptureList captureList;
  const size_t captureDataSetSize;
  const NaoJointAndSensorModel naoModel;

  /**
   * @brief operator () Almost the flavour liked by Eigen's solvers
   * @param calibrationValsStdVec testParameters, as std::vector, FULL LENGTH***
   * @param errorVec residual vector
   * @return state of the function.
   */
  int operator()(const std::vector<float> &calibrationValsStdVec,
                 Eigen::VectorXf &errorVec) const {
    auto tempModel = NaoJointAndSensorModel(naoModel);

    tempModel.setCalibValues(calibrationValsStdVec);
    //        long totalErrVecSize = 0;
    //        error.resize(capture.);
    //        rawPoseListT errorVecVec(captureList.size());

    for (size_t capIdx = 0, iter = 0; capIdx < captureList.size(); capIdx++) {
      const auto &capture = captureList[capIdx];
      //            auto & errorAtCap = errorVecVec[capIdx];
      //            errorAtCap.reserve(capture.capturedCorrespondances.size());
      const auto &camName = capture.camName;

      tempModel.setPoseCamUpdateOnly(capture.pose, capture.sf, capture.camName);
      for (const auto &correspondance : capture.capturedCorrespondances) {
        Vector2f error(0, 0);

        tempModel.robotToPixelFlt(camName, correspondance.first, error);
        error = correspondance.second - error;
        errorVec(iter++) = error.x();
        errorVec(iter++) = error.y();
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
  int operator()(const Eigen::VectorXf &calibrationVals,
                 Eigen::VectorXf &errorVec) const {
    rawPoseT calibrationValsStdVec(JOINTS::JOINT::JOINTS_MAX);
    jointCalibParamsToRawPose(calibrationVals, calibrationValsStdVec);
    return this->operator()(calibrationValsStdVec, errorVec);
  }

  /**
   * @brief operator () Simpler varient for typical Least Square
   * @param calibrationVals std::vector with test params
   * @return sum of error squared
   */
  double operator()(const std::vector<float> &calibrationVals) const {
    double out;

    auto tempModel = NaoJointAndSensorModel(naoModel);
    auto imSize = tempModel.getImSize();

    tempModel.setCalibValues(calibrationVals);
    //        long totalErrVecSize = 0;
    //        error.resize(capture.);
    //        rawPoseListT errorVecVec(captureList.size());

    for (size_t capIdx = 0; capIdx < captureList.size(); capIdx++) {
      const auto &capture = captureList[capIdx];
      //            auto & errorAtCap = errorVecVec[capIdx];
      //            errorAtCap.reserve(capture.capturedCorrespondances.size());
      const auto &camName = capture.camName;

      tempModel.setPoseCamUpdateOnly(capture.pose, capture.sf, capture.camName);
      for (const auto &correspondance : capture.capturedCorrespondances) {
        Vector2f error(0, 0);
        tempModel.robotToPixelFlt(camName, correspondance.first, error);
        error = correspondance.second - error;
        out += error.squaredNorm();
      }
    }
    return out;
  }

  /**
   * @brief JointCalibrator The constructor!
   * @param captures captures - 3d-2d correspondances
   * @param model naoJoint model
   */
  JointCalibrator(CaptureList &captures, NaoJointAndSensorModel &model)
      : captureList(captures),
        // This nasty chunk get total element count via a lambda and return to
        // the int.
        captureDataSetSize([=]() -> size_t {
          size_t temp = 0;
          for (auto &elem : captures) {
            temp += elem.capturedCorrespondances.size() * 2;
          }
          return temp;
        }()),
        naoModel(model) {}

  // Means the parameter count = 26~ in our case
  int inputs() const { return COMPACT_JOINT_CALIB_PARAM_COUNT; }
  // size of sample/ correspondance set
  size_t values() const { return captureDataSetSize; }
};

} // namespace JointCalibSolvers
