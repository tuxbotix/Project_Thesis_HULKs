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
 * @brief evalJointErrorSet Evaluate calibration capability for an error config.
 * @param naoModel As mentioned in threadedFcn..
 * @param poseList
 * @param inducedErrorStdVec Joint error configuration
 * @return pre-post residuals residuals and more infos.
 */
std::tuple<JointCalibSolvers::JointCalibResult, Eigen::VectorXf,
           Eigen::VectorXf, Eigen::VectorXf>
evalJointErrorSet(const NaoJointAndSensorModel naoModel,
                  const poseAndRawAngleListT &poseList,
                  const rawPoseT &inducedErrorStdVec, const bool &stochasticFix,
                  bool &success) {
  // Create result object
  JointCalibSolvers::JointCalibResult result;

  // Map the error vector into an Eigen vector
  const Eigen::VectorXf inducedErrorEig =
      Eigen::Map<const Eigen::VectorXf, Eigen::Unaligned>(
          inducedErrorStdVec.data(), inducedErrorStdVec.size());

  // Initialize residual of joint calibration (this isn't squared*)
  Eigen::VectorXf jointCalibResidual = inducedErrorEig;

  // Make a copy of nao sensor model
  auto naoJointSensorModel(naoModel);
  // set the error state
  naoJointSensorModel.setCalibValues(inducedErrorStdVec);

  // TODO get this from somewhere else?
  auto camNames = {Camera::BOTTOM};

  // List of captures
  JointCalibSolvers::CaptureList frameCaptures;

  // Capture per pose.
  for (auto &poseAndAngles : poseList) {
    naoJointSensorModel.setPose(poseAndAngles.angles,
                                poseAndAngles.pose.supportFoot);
    for (auto &camName : camNames) {
      bool proceed = false;
      // will be relative to support foot, easier to manage
      auto groundGrid = naoJointSensorModel.getGroundGrid(camName, proceed);
      if (proceed) {
        // world points, pixel points.
        // TODO make world points handle 3d also
        auto correspondances =
            NaoSensorDataProvider::getFilteredCorrespondancePairs(
                groundGrid,
                naoJointSensorModel.robotToPixelMulti(camName, groundGrid));
        if (correspondances.size() > 0) {
          frameCaptures.emplace_back(correspondances, poseAndAngles.angles,
                                     camName, poseAndAngles.pose.supportFoot);
        } else {
          std::lock_guard<std::mutex> lg(utils::mtx_cout_);
          std::cout << "No suitable ground points" << std::endl;
        }
      } else {
        std::lock_guard<std::mutex> lg(utils::mtx_cout_);
        std::cerr << "Obtaining ground grid failed" << std::endl;
      }
    }
  }

  // If no frames are captured, consider this as a failure
  if (frameCaptures.size() <= 0) {
    std::lock_guard<std::mutex> lg(utils::mtx_cout_);
    std::cerr << "No suitable frame captures !!" << std::endl;
    success = false;
    return {result, jointCalibResidual, Eigen::VectorXf(0), Eigen::VectorXf(0)};
  }

  /*
   * Mini eval of cost fcn to get an idea of error
   */

  auto calibrator =
      JointCalibSolvers::JointCalibrator(frameCaptures, naoJointSensorModel);

  rawPoseT calVec(JOINTS::JOINT::JOINTS_MAX, 0.0);

  Eigen::VectorXf errorVec(calibrator.values());
  Eigen::VectorXf finalErrorVec(calibrator.values());

  calibrator(Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(calVec.data(),
                                                           calVec.size()),
             errorVec);

  /*
   * Run Lev-Mar to optimize
   */
  auto runLevMar = [&calibrator](Eigen::VectorXf &params) {
    Eigen::NumericalDiff<JointCalibSolvers::JointCalibrator> functorDiff(
        calibrator);
    Eigen::LevenbergMarquardt<
        Eigen::NumericalDiff<JointCalibSolvers::JointCalibrator>, float>
        lm(functorDiff);

    int status = lm.minimize(params);
    return status;
  };

  // Setting zero is very important.
  Eigen::VectorXf calibratedParams((int)JOINTS::JOINT::JOINTS_MAX);
  calibratedParams.setZero();

  /// Run the Levenberg-Marquardt solver
  int status = runLevMar(calibratedParams);
  //  Get reprojection error
  calibrator(calibratedParams, finalErrorVec);

  // Just to note if the loop was done and broken with success
  bool loopedAndBroken = false;
  // Do stochastic fixing?
  if (stochasticFix) {
    // This will hold best params in case looping is needed
    Eigen::VectorXf oldParams((int)JOINTS::JOINT::JOINTS_MAX);
    size_t count = 0;                     // counter for the while loop
    std::default_random_engine generator; // random gen
    /// TODO make this distribution configurable
    std::uniform_real_distribution<float> distribution(-5, 5);

    /*
     * If status is 2 or 5, most likely we are at a local minima
     * So to give some push-start random start positions will be attempted
     */
    while ((status == 2 || status == 5) && count < 100) {
      // save state
      oldParams = calibratedParams;
      // get cost
      double finalCost = finalErrorVec.squaredNorm();

      /// Populate the starting point with random values

      calibratedParams(JOINTS::JOINT::HEAD_PITCH) = std::min(
          0.0f, static_cast<float>(distribution(generator)) / 2 * TO_RAD);
      calibratedParams(JOINTS::JOINT::HEAD_YAW) =
          distribution(generator) * TO_RAD;

      // todo make this dual leg supported..
      for (size_t i = JOINTS::JOINT::L_HIP_YAW_PITCH;
           i <= JOINTS::JOINT::L_ANKLE_ROLL; i++) {
        calibratedParams(i) = distribution(generator) * TO_RAD;
      }
      /// Run Lev-Mar again
      status = runLevMar(calibratedParams);
      // Get the new cost
      calibrator(calibratedParams, finalErrorVec);

      // Check status and continue accordingly
      if (status == 4 && finalCost > finalErrorVec.squaredNorm()) {
        //      std::lock_guard<std::mutex> lg(utils::mtx_cout_);
        //      std::cout << "break: " << (finalCost <
        //      finalErrorVec.squaredNorm()) << std::endl;
        loopedAndBroken = true;
        break;
      } else if (
          //                    (status == 2 || status == 5) ||
          finalCost < finalErrorVec.squaredNorm()) {
        calibratedParams = oldParams;
      }
      count++;
    }
  }

  /*
   * END lev-mar
   */

  jointCalibResidual -= calibratedParams; // Joint residual
  const float tolerance = 0.5f * TO_RAD;  // Ad-hoc tolerance

  // Get min-max info for error checking
  auto minCoeff = finalErrorVec.minCoeff();
  auto maxCoeff = finalErrorVec.maxCoeff();
  auto finalErrAbsMax = std::max(std::abs(minCoeff), std::abs(maxCoeff));
  auto errorNorm = finalErrorVec.norm();
  auto errorAvg = finalErrorVec.norm() / finalErrorVec.size();
  auto jointCalibResAbsMax = std::max(std::abs(jointCalibResidual.maxCoeff()),
                                      std::abs(jointCalibResidual.minCoeff()));
  const auto imageSize = naoModel.getImSize();

  // Determine success or failure
  if (std::isnan(minCoeff) || std::isnan(maxCoeff) ||
      finalErrAbsMax > imageSize.minCoeff() * 0.1 ||
      errorNorm > errorVec.norm()) {
    std::cout << "Calibration failure\n"
              << calibratedParams.transpose() << "\n"
              << inducedErrorEig.transpose() << "\n"
              << finalErrorVec.norm() << std::endl;
    success = false;
  } else {
    success = true;
  }

  // if failed OR if loop broken BUT calibration is bad
  /*
   * NOTE: Local minima if reprojection errr is small but jointResidual isn't
   * low!
   */
  //  if (!success ||
  //      (stochasticFix && loopedAndBroken && jointCalibResAbsMax > tolerance))
  //      {

  std::lock_guard<std::mutex> lg(utils::mtx_cout_);
  std::cout << "errAbsMax: " << std::max(std::abs(minCoeff), std::abs(maxCoeff))
            << " "
            << " errorNorm: " << jointCalibResAbsMax
            << " errorAvg: " << jointCalibResAbsMax
            << " jointCalibResAbsMax: " << jointCalibResAbsMax / TO_RAD
            << " Status: " << status << " stochastic & success?: "
            << (stochasticFix && loopedAndBroken &&
                jointCalibResAbsMax <= tolerance)
            << std::endl;
  //  }

  // Update the result object
  result.status = status;
  result.jointParams.resize(JOINTS::JOINT::JOINTS_MAX);
  for (int i = 0; i < JOINTS::JOINT::JOINTS_MAX; ++i) {
    result.jointParams[i] = calibratedParams(i);
  }
  result.reprojectionErrorNorm = errorNorm;
  result.reprojectionErrorNorm = errorAvg;

  return {result, jointCalibResidual, errorVec, finalErrorVec};
}

int main(int argc, char *argv[]) {
  std::string inFileName((argc > 1 ? argv[1] : "out"));
  // Default is true..
  bool stochasticFix =
      argc > 2 ? (std::string(argv[2]).compare("s") == 0) : true;
  std::string confRoot((argc > 3 ? argv[3] : "../../nao/home/"));

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
  std::uniform_real_distribution<float> distribution(-0.5, 0.5);
  //  std::normal_distribution<float> distribution(0.0, 2);

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
    auto residuals = evalJointErrorSet(naoJointSensorModel, poseList, errorSet,
                                       stochasticFix, success);
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
