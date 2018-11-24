#include <iostream>
// #include <iomanip>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <random>
#include <sstream>
#include <thread>
#include <unordered_set>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <unsupported/Eigen/NonLinearOptimization>

#include <dlib/global_optimization.h>

#include <Data/CameraMatrix.hpp>
#include <Modules/Configuration/Configuration.h>
#include <Modules/Configuration/UnixSocketConfig.hpp>
#include <Modules/NaoProvider.h>
#include <Modules/Poses.h>

#include <Hardware/RobotInterface.hpp>
#include <Tools/Kinematics/KinematicMatrix.h>
#include <Tools/Storage/Image.hpp>
#include <Tools/Storage/Image422.hpp>

#define DEBUG_JOINT_AND_SENS 0

#include "NaoJointAndSensorModel.hpp"
#include "NaoPoseInfo.hpp"
#include "NaoStability.hpp"
#include "NaoTorsoPose.hpp"
#include "TUHHMin.hpp"
#include "constants.hpp"

#include "JointCalibSolver.hpp"

#include "ObservationSensitivityProvider.hpp"

#define DO_COMMIT 1
#define WRITE_PARALLEL 1
#define DEBUG_FILE_COMMIT 1
#define DEBUG_IN_THREAD 1
#define ENABLE_PROGRESS 1

#include "utils.hpp"

const unsigned int MAX_THREADS = std::thread::hardware_concurrency();
const unsigned int MAX_POSES_TO_CALIB = 10;

/**
 * @brief evalJointErrorSet
 * @param naoModel
 * @param poseList
 * @param inducedErrorStdVec
 * @return pre-post residuals
 */
std::pair<Eigen::VectorXf, Eigen::VectorXf>
evalJointErrorSet(const NaoJointAndSensorModel naoModel,
                  const poseAndRawAngleListT &poseList,
                  const rawPoseT &inducedErrorStdVec, bool &success) {

  auto naoJointSensorModel(naoModel);
  naoJointSensorModel.setCalibValues(inducedErrorStdVec);

  auto camNames = {Camera::BOTTOM};

  JointCalibSolvers::CaptureList frameCaptures;

  for (auto &poseAndAngles : poseList) {
    naoJointSensorModel.setPose(poseAndAngles.angles,
                                poseAndAngles.pose.supportFoot);
    for (auto &camName : camNames) {
      bool success = false;
      // will be relative to support foot, easier to manage
      auto groundGrid = naoJointSensorModel.getGroundGrid(camName, success);
      if (success) {
        // world points, pixel points.
        // TODO make world points handle 3d also
        auto pixelMulti =
            naoJointSensorModel.robotToPixelMulti(camName, groundGrid);
        auto correspondances =
            NaoSensorDataProvider::getFilteredCorrespondancePairs(groundGrid,
                                                                  pixelMulti);
        if (correspondances.size() > 0) {
          frameCaptures.emplace_back(correspondances, poseAndAngles.angles,
                                     camName, poseAndAngles.pose.supportFoot);
        } else {
          std::cout << "No suitable ground points" << std::endl;
        }
        //                std::cout << " % successful correspondances : " <<
        //                (float)correspondances.size()/
        //                (float)pixelMulti.size() << " " <<
        //                correspondances.size() << " " << pixelMulti.size()  <<
        //                std::endl;
      } else {
        std::cerr << "Obtaining ground grid failed" << std::endl;
      }
    }
  }

  if (frameCaptures.size() <= 0) {
    std::cout << "No suitable frame captures !!" << std::endl;
    success = false;
    return {Eigen::VectorXf(0), Eigen::VectorXf(0)};
  }
  std::cout << "Initiate calibrator " << std::endl;
  /*
   * Mini eval of cost fcn
   */

  auto calibrator =
      JointCalibSolvers::JointCalibrator(frameCaptures, naoJointSensorModel);

  rawPoseT calVec(JOINTS::JOINT::JOINTS_MAX, 0.0);

  const Eigen::VectorXf inducedErrorEig =
      Eigen::Map<const Eigen::VectorXf, Eigen::Unaligned>(
          inducedErrorStdVec.data(), inducedErrorStdVec.size());

  Eigen::VectorXf errorVec(calibrator.values());
  Eigen::VectorXf finalErrorVec(calibrator.values());

  //    std::cout<< "Compensated Calib state: " << calibrator(inducedErrorEig,
  //    errorVec) << " cost: " << errorVec.norm() << std::endl;

  //    {
  //        std::lock_guard<std::mutex> lg(utils::mtx_cout_);
  //    std::cout<< "Calib state pre: " <<
  calibrator(Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(calVec.data(),
                                                           calVec.size()),
             errorVec);
  //                        << " cost: " << errorVec.norm() << " avg err " <<
  //                        errorVec.sum();
  //    }

  Eigen::NumericalDiff<JointCalibSolvers::JointCalibrator> functorDiff(
      calibrator);
  Eigen::LevenbergMarquardt<
      Eigen::NumericalDiff<JointCalibSolvers::JointCalibrator>, float>
      lm(functorDiff);

  /*
   * EIGEN SOLVER
   */
//   Important bit!! Initialize AND fill! :P
    Eigen::VectorXf calibratedParamsEig;
    calibratedParamsEig.resize(JOINTS::JOINT::JOINTS_MAX);
    calibratedParamsEig.setZero();

    int status = lm.minimize(calibratedParamsEig);
    calibrator(calibratedParamsEig, finalErrorVec);
  /*
   * DLIB SOLVER
   */
//  std::vector<float> calibratedParams; //(JOINTS::JOINT::JOINTS_MAX, 0.0);


//  //  auto calibrator
//  //      dlib::fin=_min_global()

//  auto holder_table = [calibrator](dlib::matrix<double, 0, 1> x0) -> double {
//    if (x0.size() >= JOINTS::JOINT::JOINTS_MAX) {
//      std::vector<float> ll(x0.begin(), x0.end());
//      return calibrator(ll);
//    } else {
//      std::cout << x0.size() << "we got a problem!" << std::endl;
//    }
//    return 0.0;
//  };

//  dlib::matrix<double, 0, 1> boundUpper =
//      dlib::uniform_matrix<double>(JOINTS::JOINT::JOINTS_MAX, 1, 10 * TO_RAD);
//  dlib::matrix<double, 0, 1> solution =
//      dlib::uniform_matrix<double>(JOINTS::JOINT::JOINTS_MAX, 1, 0 * TO_RAD);

//  //    std::cout<<boundUpper.size() << " " << boundUpper<<std::endl;
//  // obtain result.x and result.y
//  //  auto result = dlib::find_min_global(holder_table, -boundUpper, // lower
//  //  bounds
//  //                                      boundUpper,                 // upper
//  //                                      bounds
//  //                                      dlib::max_function_calls(200));
////  solution = result.x;

////  auto result =
//          dlib::find_min_box_constrained(
//      dlib::lbfgs_search_strategy(10), dlib::objective_delta_stop_strategy(1e-10),
//      holder_table, dlib::derivative(holder_table), solution, -10, 10);


////  std::cout << result.y << std::endl;
//  //    std::cout<< "Calib status: "<< status <<
//  //                "\nInduced Error and calib diff: \n" << ((inducedErrorEig -
//  //                calibratedParams)/ TO_RAD).transpose() <<
//  //                "\nCalibration values:   \n" << (calibratedParams /
//  //                TO_RAD).transpose() <<std::endl;
//  calibratedParams = std::vector<float>(solution.begin(), solution.end());
//  const Eigen::VectorXf calibratedParamsEig =
//      Eigen::Map<const Eigen::VectorXf, Eigen::Unaligned>(
//          calibratedParams.data(), calibratedParams.size());
//calibrator(calibratedParams, finalErrorVec);
    /*
     * END DLIB
     */


  std::cout << "init reproj: cost: " << errorVec.norm() << " avg err "
            << errorVec.sum() << std::endl;

  //    auto imSize = naoJointSensorModel.getImSize();
  auto minCoeff = finalErrorVec.minCoeff();
  auto maxCoeff = finalErrorVec.maxCoeff();
  //    auto imMaxX2 = std::max(imSize.x(), imSize.y()) * 2;

  /*if(std::isnan(minCoeff) || std::isnan(maxCoeff) || std::abs(minCoeff) >
imMaxX2 || std::abs(maxCoeff) > imMaxX2)
  {
      std::lock_guard<std::mutex> lg(utils::mtx_cout_);
//        std::cout<< " post: " << status << " " << "min: " <<
finalErrorVec.minCoeff() << " max: " << finalErrorVec.maxCoeff() << std::endl;
//        std::cout << inducedErrorEig.transpose() / TO_RAD << std::endl;
      success = false;
  }else */
  if (std::isnan(minCoeff) || std::isnan(maxCoeff) ||
      finalErrorVec.norm() > errorVec.norm()) {
    std::cout << "Calibration failure" << errorVec.norm() << " "
              << finalErrorVec.norm() << " "
              << "min: " << finalErrorVec.minCoeff()
              << " max: " << finalErrorVec.maxCoeff() << std::endl;
    success = false;
  } else {
    success = true;
  }
  std::cout << "reproj: cost: " << finalErrorVec.norm() << " avg err "
            << finalErrorVec.sum() << std::endl;

  auto paramErr = (inducedErrorEig - calibratedParamsEig);
  std::cout << "param: cost: " << paramErr.norm() << " avg err "
            << paramErr.sum() / paramErr.size() << std::endl;

  //    std::cout<< "Dumping residuals" << std::endl;

  return {errorVec, finalErrorVec};
  //    std::pair<Eigen::VectorXf &, std::string> originalResidualDump(errorVec,
  //    inFileName + ".originalResidual");
  //    std::pair<Eigen::VectorXf &, std::string>
  //    calibratedResidualDump(finalErrorVec, inFileName +
  //    ".calibratedResidual");

  //    for(auto elem : {originalResidualDump, calibratedResidualDump}){
  //        std::cout<< "Start dump to : " << elem.second << std::endl;
  //        std::fstream file(elem.second, std::ios::out);
  //        if(file){
  //            auto & residualVec = elem.first;
  //            for(long i = 0; i < residualVec.size(); ++i){
  //                if(i % 2 == 0){ // even number
  //                    file << residualVec(i);
  //                }else{ // odd  -> newline
  //                    file << "," << residualVec(i) << "\n";
  //                }
  //            }
  //        }
  //    }
}

int main(int argc, char *argv[]) {
  std::string inFileName((argc > 1 ? argv[1] : "out"));
  //    std::string inFileName((argc > 1 ? argv[1] : "out"));
  std::string confRoot((argc > 2 ? argv[2] : "../../nao/home/"));

  std::fstream inFile(inFileName, std::ios::in);
  if (!inFile.is_open()) {
    std::cout << "input isnt here" << std::endl;
    return 1;
  }
  /*
   * Initializing model
   */
  TUHH tuhhInstance(confRoot);

  Vector2f fc, cc, fov;

  tuhhInstance.config_.mount("Projection", "Projection.json",
                             ConfigurationType::HEAD);
  tuhhInstance.config_.get("Projection", "top_fc") >> fc;
  tuhhInstance.config_.get("Projection", "top_cc") >> cc;
  tuhhInstance.config_.get("Projection", "fov") >> fov;

  Vector2i imSize(640, 480);

  CameraMatrix camMat;
  camMat.fc = fc;
  camMat.fc.x() *= imSize.x();
  camMat.fc.y() *= imSize.y();
  camMat.cc = cc;
  camMat.cc.x() *= imSize.x();
  camMat.cc.y() *= imSize.y();
  camMat.fov = fov;

  poseAndRawAngleListT poseList;
  poseList.reserve(MAX_POSES_TO_CALIB);
  poseAndRawAngleT poseAndAngles;

  for (size_t i = 0; i < MAX_POSES_TO_CALIB &&
                     utils::JointsAndPosesStream::getNextPoseAndRawAngles(
                         inFile, poseAndAngles);
       ++i) {
    poseList.push_back(poseAndAngles);
  }

  std::cout << " reading of " << poseList.size() << " poses done" << std::endl;

  /*
   * Make dataset
   */

  // TODO check if uniform distribution is the right choice?
  std::default_random_engine generator;
  //    std::uniform_real_distribution<float> distribution(-0.5, 0.5);
  std::normal_distribution<float> distribution(0.0, 2);

  const size_t JOINT_ERR_LST_DESIRED_COUNT = 20;

  std::set<rawPoseT> uniqueJointErrList;
  //    std::vector<rawPoseT> jointErrList(JOINT_ERR_LST_DESIRED_COUNT);
  for (size_t iter = 0; iter < JOINT_ERR_LST_DESIRED_COUNT; iter++) {
    rawPoseT elem = rawPoseT(JOINTS::JOINT::JOINTS_MAX, 0.0f);

    //        elem[ JOINTS::JOINT::HEAD_PITCH] = std::min(0.0f,
    //        static_cast<float>(distribution(generator))/2 * TO_RAD);
    //        elem[ JOINTS::JOINT::HEAD_YAW] = distribution(generator) * TO_RAD;
    //        elem[ JOINTS::JOINT::HEAD_PITCH] =10 * TO_RAD;

    // todo make this dual leg supported..
    for (size_t i = JOINTS::JOINT::L_ANKLE_PITCH;
         i <= JOINTS::JOINT::L_ANKLE_PITCH; i++) {
      elem[i] = distribution(generator) * TO_RAD;
      //                  elem[i] = 10 * TO_RAD;
    }
    uniqueJointErrList.insert(elem);
  }

  const size_t JOINT_ERR_LST_COUNT = uniqueJointErrList.size();
  std::vector<rawPoseT> jointErrList(JOINT_ERR_LST_COUNT);
  std::copy(uniqueJointErrList.begin(), uniqueJointErrList.end(),
            jointErrList.begin());
  uniqueJointErrList.clear();

  std::cout << jointErrList.size() << "error lists to try" << std::endl;

  // create nao model
  const ObservationModelConfig cfg = {
      imSize, fc, cc, fov, {SENSOR_NAME::BOTTOM_CAMERA}, 1000, 50, 0.05};
  auto naoJointSensorModel = NaoJointAndSensorModel(
      imSize, fc, cc, fov, cfg.maxGridPointsPerSide, cfg.gridSpacing);

  // start calib runs
  size_t badCases = 0;

  std::cout << " Running calibrations" << std::endl;

  //    Vector2f min = {1000.0f, 1000.0f } , max = {-1000.0f, -1000.0f};

  for (size_t i = 0; i < JOINT_ERR_LST_COUNT; i++) {
    auto &errorSet = jointErrList[i];
    bool success = false;
    auto residuals =
        evalJointErrorSet(naoJointSensorModel, poseList, errorSet, success);
    if (!success) {
      ++badCases;
      continue;
    }

    //        for(long i = 0; i < residuals.first.size(); ++i){
    //           if(i % 2 == 0){ // x -> even number
    //               preResidualsXY.first.push_back(residuals.first(i));
    //               postResidualsXY.first.push_back(residuals.second(i));

    //               min.x() = std::min(residuals.second(i), min.x());
    //               max.x() = std::max(residuals.second(i), max.x());
    //           }else{ // odd  -> newline
    //               preResidualsXY.second.push_back(residuals.first(i));
    //               postResidualsXY.second.push_back(residuals.second(i));

    //               min.y() = std::min(residuals.second(i), min.y());
    //               max.y() = std::max(residuals.second(i), max.y());
    //           }
    //        }

    if ((size_t)(JOINT_ERR_LST_COUNT * 0.01)) {
      if (i % (size_t)(JOINT_ERR_LST_COUNT * 0.01) == 0) {
        std::cout << "list: " << i
                  << " badCases: " << badCases * 100 / JOINT_ERR_LST_COUNT
                  << std::endl;
      }
    }
  }

  return 0;
}
