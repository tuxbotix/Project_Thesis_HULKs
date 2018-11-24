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

//#include <Data/CameraMatrix.hpp>
//#include <Modules/NaoProvider.h>
//#include <Modules/Configuration/Configuration.h>
//#include <Modules/Configuration/UnixSocketConfig.hpp>
//#include <Modules/Poses.h>

//#include <Tools/Storage/Image.hpp>
//#include <Tools/Storage/Image422.hpp>
//#include <Tools/Kinematics/KinematicMatrix.h>
//#include <Hardware/RobotInterface.hpp>

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

// struct JointCalibCapture{
//    NaoPose<float> pose;
//    Correspondance3D2DList capturedCorrespondances;
//};

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

struct JointCalibResult {
  int status; // out of solver
  float reprojectionErrorNorm;
  float reprojectionErrorAvg;
  rawPoseT jointParams; // solved params
};

typedef std::vector<JointCalibCaptureEvalT> CaptureList;

/*
 * Below is DLib based global optimizer..
 */

/*
 * Below is Eigen based non-global optimization
 */

///**
// * Extracted from
// https://github.com/daviddoria/Examples/blob/master/c%2B%2B/Eigen/LevenbergMarquardt/CurveFitting.cpp
// * This definition structure is needed to use NumericalDiff module
// */

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

struct JointCalibrator : Functor<float> {

  const CaptureList captureList;
  const size_t captureDataSetSize;
  const NaoJointAndSensorModel naoModel;

  int operator()(const std::vector<float> &calibrationValsStdVec,
                 Eigen::VectorXf &errorVec) const {
    auto tempModel = NaoJointAndSensorModel(naoModel);
    auto imSize = tempModel.getImSize();

    //        rawPoseT calibrationValsStdVec;
    //        calibrationValsStdVec.resize(calibrationVals.size());
    //        Eigen::VectorXf::Map(&calibrationValsStdVec[0],
    //        calibrationVals.size()) = calibrationVals;

    //        for(int i =0; i< JOINTS::JOINT::JOINTS_MAX;++i){
    //            std::cout << calibrationVals.transpose()<<std::endl;
    //        }

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
        //                if(
        tempModel.robotToPixelFlt(camName, correspondance.first, error);
        //                        ){

        error = correspondance.second - error;
        //                    if(std::abs(error.x()) > imSize.x()){
        //                        errorAtCap.push_back(imSize.x() *
        //                        utils::sgn(error.x()));
        //                    }else{

        errorVec(iter++) = error.x();
        errorVec(iter++) = error.y();

        //                        errorAtCap.push_back(error.x());
        //                    }
        //                    if(std::abs(error.y()) > imSize.y()){
        //                        errorAtCap.push_back(imSize.y() *
        //                        utils::sgn(error.y()));
        //                    }else{
        //                        errorAtCap.push_back(error.y());
        //                    }

        //                    totalErrVecSize++;
        //                }else{
        //                    std::cout << "behind " <<std::endl;
        //                    std::cout << error.transpose() << " " <<
        //                    correspondance.second.transpose() <<std::endl;
        //                    errorAtCap.push_back(0);
        //                    errorAtCap.push_back(0);
        //                    errorVec(iter++) = 0;
        //                    errorVec(iter++) = 0;
        //                }
      }
    }
    return 0;
  }

  int operator()(const Eigen::VectorXf &calibrationVals,
                 Eigen::VectorXf &errorVec) const {
    rawPoseT calibrationValsStdVec(static_cast<size_t>(calibrationVals.size()));
    Eigen::VectorXf::Map(&calibrationValsStdVec[0], calibrationVals.size()) =
        calibrationVals;
    return this->operator()(calibrationValsStdVec, errorVec);
  }

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
  int inputs() const { return JOINTS::JOINT::JOINTS_MAX; }
  // size of sample/ correspondance set
  size_t values() const { return captureDataSetSize; }
};

} // namespace ObservationSolvers
