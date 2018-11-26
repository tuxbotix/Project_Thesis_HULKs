#include <iostream>
#include <tuple>
// #include <iomanip>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <forward_list>
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

#include "NaoJointAndSensorModel.hpp"
#include "NaoPoseInfo.hpp"
#include "NaoStability.hpp"
#include "NaoTorsoPose.hpp"
#include "TUHHMin.hpp"
#include "constants.hpp"

#define DEBUG_CAM_OBS 1

#include "CameraObservationModel.hpp"
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

template <class T> using Residual = std::vector<T>;

template <typename T> struct CalibEvalResiduals {
  Residual<T> preX;
  Residual<T> preY;
  Residual<T> postX;
  Residual<T> postY;

  std::array<Residual<T>, JOINTS::JOINT::JOINTS_MAX> jointResiduals;

  /**
   * @brief joinEvalResiduals move-insert ops for all vectors. Src list is
   * joined to dest list
   * @param src source CalibEvalResidual
   * @param dest destination CalibEvalResidual
   * @return Reference to dest
   */
  CalibEvalResiduals &joinEvalResiduals(CalibEvalResiduals &src,
                                        CalibEvalResiduals &dest) {
    dest.preX.insert(dest.preX.end(), std::make_move_iterator(src.preX.begin()),
                     std::make_move_iterator(src.preX.end()));
    dest.preY.insert(dest.preY.end(), std::make_move_iterator(src.preY.begin()),
                     std::make_move_iterator(src.preY.end()));
    dest.postX.insert(dest.postX.end(),
                      std::make_move_iterator(src.postX.begin()),
                      std::make_move_iterator(src.postX.end()));
    dest.postY.insert(dest.postY.end(),
                      std::make_move_iterator(src.postY.begin()),
                      std::make_move_iterator(src.postY.end()));
    for (size_t i = 0; i < JOINTS::JOINT::JOINTS_MAX; ++i) {
      auto &srcjoint = src.jointResiduals[i];
      auto &dstjoint = dest.jointResiduals[i];
      dstjoint.insert(dstjoint.end(), std::make_move_iterator(srcjoint.begin()),
                      std::make_move_iterator(srcjoint.end()));
    }
    return dest;
  }
};

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
    std::uniform_real_distribution<float> distribution(-6.5, 6.5);

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
        // Get the new cost
        calibrator(calibratedParams, finalErrorVec);
      }
      count++;
    }
  }

  /// Run Lev-Mar again
  status = runLevMar(calibratedParams);

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
      errorNorm > errorVec.norm() || jointCalibResAbsMax > tolerance) {
    //    std::lock_guard<std::mutex> lg(utils::mtx_cout_);
    //    std::cout << "Calibration failure\n"
    //              << calibratedParams.transpose() << "\n"
    //              << inducedErrorEig.transpose() << "\n"
    //              << finalErrorVec.norm() << std::endl;
    success = false;
  } else {
    success = true;
  }

  // if failed OR if loop broken BUT calibration is bad
  /*
   * NOTE: Local minima if reprojection errr is small but jointResidual isn't
   * low!
   */
  if (!success ||
      (stochasticFix && loopedAndBroken && jointCalibResAbsMax > tolerance)) {

    std::lock_guard<std::mutex> lg(utils::mtx_cout_);
    std::cout << "errAbsMax: "
              << std::max(std::abs(minCoeff), std::abs(maxCoeff)) << " "
              << " errorNorm: " << jointCalibResAbsMax
              << " errorAvg: " << jointCalibResAbsMax
              << " jointCalibResAbsMax: " << jointCalibResAbsMax / TO_RAD
              << " Status: " << status << std::endl;
  }

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

/**
 * @brief threadedFcn This function evaluate calibration of a set of errors
 * given
 * @param residuals An object containing vectors that hold various residuals
 * @param poseList List of possible poses for calibration
 * @param naoJointSensorModel Nao's simulated model which is under test
 * @param jointErrItrBegin begin iterator of jointErrorVector
 * @param jointErrItrEnd end iterator of jointErrorVector
 * @param stochasticFix Flag to enable or disable stochastic improving
 * @param badCases Count of failed calibrations.
 */
void threadedFcn(CalibEvalResiduals<double> &residuals,
                 const poseAndRawAngleListT &poseList,
                 const NaoJointAndSensorModel &naoJointSensorModel,
                 const std::vector<rawPoseT>::iterator jointErrItrBegin,
                 const std::vector<rawPoseT>::iterator jointErrItrEnd,
                 bool stochasticFix, std::atomic<size_t> &badCases) {

  size_t tempBadCases = 0; // temp storage
  for (auto errSetIter = jointErrItrBegin; errSetIter != jointErrItrEnd;
       ++errSetIter) {

    auto &errorSet = *errSetIter;

    bool success = false; // change this to more detailed output

    JointCalibSolvers::JointCalibResult result;
    Eigen::VectorXf preResiduals;
    Eigen::VectorXf postResiduals;
    Eigen::VectorXf jointCalibResiduals;
    // Call evaluator for this error config.
    std::tie(result, jointCalibResiduals, preResiduals, postResiduals) =
        evalJointErrorSet(naoJointSensorModel, poseList, errorSet,
                          stochasticFix, success);

    if (!success) {
      // if a failure, update the counter.
      ++tempBadCases;
      //      continue;
    }

    auto prePostResSize = static_cast<size_t>(preResiduals.size());
    auto curPreResSize = residuals.preX.size();

    // reserve vector memory to speed up the process
    residuals.preX.reserve(prePostResSize + curPreResSize);
    residuals.postX.reserve(prePostResSize + curPreResSize);
    residuals.preY.reserve(prePostResSize + curPreResSize);
    residuals.postY.reserve(prePostResSize + curPreResSize);

    for (long i = 0; i < preResiduals.size(); ++i) {
      if (i % 2 == 0) { // x -> even number
        residuals.preX.push_back(preResiduals(i));
        residuals.postX.push_back(postResiduals(i));
      } else { // odd  -> newline
        residuals.preY.push_back(preResiduals(i));
        residuals.postY.push_back(postResiduals(i));
      }
    }
    // Convert to degrees
    jointCalibResiduals /= TO_RAD;
    for (long i = 0; i < JOINTS::JOINT::JOINTS_MAX; ++i) {
      // TODO FIX THIS
      //      residuals.jointResiduals[static_cast<size_t>(i)].push_back(
      //          jointCalibResiduals(i));
      residuals.jointResiduals[0].push_back(jointCalibResiduals(i));
    }
  }
  badCases += tempBadCases;
}

/**
 * @brief main
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char *argv[]) {
  std::string inFileName((argc > 1 ? argv[1] : "out"));
  // Default is true..
  bool stochasticFix =
      argc > 2 ? (std::string(argv[2]).compare("s") == 0) : true;
  std::string confRoot((argc > 3 ? argv[3] : "../../nao/home/"));

  std::fstream inFile(inFileName);
  if (!inFile.is_open()) {
    std::cerr << "Input file cannot be opened, exiting.." << std::endl;
    return 1;
  }

  /*
   * Initializing model & Nao provider
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

  // create nao model TODO change camera mode?
  const ObservationModelConfig cfg = {
      imSize, fc, cc, fov, {SENSOR_NAME::BOTTOM_CAMERA}, 1000, 50, 0.05};
  auto naoJointSensorModel = NaoJointAndSensorModel(
      imSize, fc, cc, fov, cfg.maxGridPointsPerSide, cfg.gridSpacing);

  /*
   * Read poses
   */
  poseAndRawAngleListT poseList;
  poseList.reserve(MAX_POSES_TO_CALIB);
  poseAndRawAngleT poseAndAngles;

  for (size_t i = 0; i < MAX_POSES_TO_CALIB &&
                     utils::JointsAndPosesStream::getNextPoseAndRawAngles(
                         inFile, poseAndAngles);
       ++i) {
    poseList.push_back(poseAndAngles);
  }
  std::cout << " reading of poses done" << std::endl;

  /*
   * Make dataset (joint errors)
   */

  // TODO check if uniform distribution is the right choice?
  std::default_random_engine generator;
  const float MAX_ERR_VAL = 6.5;
  const float MIN_ERR_VAL = -MAX_ERR_VAL;
  /// New change, try 1 to 6, -1 to -6 only
  std::uniform_real_distribution<float> distribution(MIN_ERR_VAL, MAX_ERR_VAL);

  //  std::uniform_int_distribution<int> binDistribution(0, 1);
  //  const auto plusOrMinus = [&binDistribution, &generator]() -> float {
  //    return binDistribution(generator) ? 1.0f : -1.0f;
  //  };
  //  std::normal_distribution<float> distribution(0.0, 2);

  /// Sample Size
  const size_t JOINT_ERR_LST_DESIRED_COUNT = 1000;

  std::set<rawPoseT> uniqueJointErrList;

  for (size_t iter = 0; iter < JOINT_ERR_LST_DESIRED_COUNT; iter++) {
    rawPoseT elem = rawPoseT(JOINTS::JOINT::JOINTS_MAX, 0.0f);

    /// Do head angles seperately..
    elem[JOINTS::JOINT::HEAD_PITCH] = std::min(
        0.0f, static_cast<float>(distribution(generator)) / 2 * TO_RAD);
    elem[JOINTS::JOINT::HEAD_YAW] =
        distribution(generator) * TO_RAD /* * plusOrMinus() */;

    /// Leg Angles
    // TODO make this dual leg supported..
    for (size_t i = JOINTS::JOINT::L_HIP_YAW_PITCH;
         i <= JOINTS::JOINT::L_ANKLE_ROLL; i++) {
      elem[i] = distribution(generator) * TO_RAD /* * plusOrMinus() */;
    }
    uniqueJointErrList.insert(elem);
  }

  /*
   * Ensure the dataSet is unique
   */
  const size_t JOINT_ERR_LST_COUNT = uniqueJointErrList.size();
  std::vector<rawPoseT> jointErrList(JOINT_ERR_LST_COUNT);
  std::copy(uniqueJointErrList.begin(), uniqueJointErrList.end(),
            jointErrList.begin());
  uniqueJointErrList.clear();
  std::cout << jointErrList.size() << "error lists to try" << std::endl;

  /*
   * Prepare for multithreading and start calibration
   */
  std::cout << "Starting calibrations" << std::endl;

  // total count of "bad cases"
  std::atomic<size_t> badCases;
  badCases = 0;

  // VecOfVec of CalibEvalResiduals - they contain image and joint residual info
  // + others
  std::vector<CalibEvalResiduals<double>> calibResidualList(MAX_THREADS);
  // This will contain all above at the end. (move-join)
  CalibEvalResiduals<double> finalResidualSet;

  // vector holding the std::threads
  std::vector<std::thread> threadList(MAX_THREADS);

  // Setup offsets for iterators
  size_t threadingOffsets = JOINT_ERR_LST_COUNT / MAX_THREADS;

  // Start the threads
  for (size_t i = 0; i < MAX_THREADS; ++i) {

    std::vector<rawPoseT>::iterator begin =
        jointErrList.begin() + (threadingOffsets * i);
    std::vector<rawPoseT>::iterator end =
        (i + 1 >= MAX_THREADS)
            ? jointErrList.end()
            : jointErrList.begin() + threadingOffsets * (i + 1);

    std::cout << "starting thread: " << i << std::endl;
    // Initialize the thread. Maybe use futures later?
    threadList[i] =
        std::thread(threadedFcn, std::ref(calibResidualList[i]),
                    std::ref(poseList), std::ref(naoJointSensorModel), begin,
                    end, stochasticFix, std::ref(badCases));
  }

  /*
   * Finish calibration - join threads
   * Also join all residuals into finalResidualSet
   */
  for (size_t i = 0; i < MAX_THREADS; ++i) {
    auto &thread = threadList[i];
    auto &residual = calibResidualList[i];
    if (thread.joinable()) {
      thread.join();
    }
    std::cout << "Going to join" << i << std::endl;
    finalResidualSet.joinEvalResiduals(residual, finalResidualSet);
  }
  std::cout << "Processing done" << std::endl;

  /*
   * Post-processing -> Get min-max bounds
   */

  std::cout << " initializing histograms " << std::endl;
  auto minMaxPostX = std::minmax_element(finalResidualSet.postX.begin(),
                                         finalResidualSet.postX.end());

  auto minMaxPostY = std::minmax_element(finalResidualSet.postY.begin(),
                                         finalResidualSet.postY.end());
  auto minMaxJointParams =
      std::minmax_element(finalResidualSet.jointResiduals[0].begin(),
                          finalResidualSet.jointResiduals[0].end());
  auto maxOfJointParamsAll =
      1.01 * std::max(std::abs(*minMaxJointParams.first),
                      std::abs(*minMaxJointParams.second));

  auto maxOfAll = std::max(std::abs(*minMaxPostX.first), *minMaxPostX.second);
  maxOfAll = 1.01 * std::max(std::max(std::abs(*minMaxPostY.first),
                                      *minMaxPostY.second),
                             maxOfAll);

  std::cout << "absMax Post Errors" << maxOfAll << std::endl;
  std::cout << "absMax Joint Errors" << maxOfJointParamsAll << std::endl;

  /*
   * Initialize Histograms
   */
  utils::SimpleHistogram<double> preXhist(1000, -300, 300);
  utils::SimpleHistogram<double> preYhist(1000, -300, 300);

  utils::SimpleHistogram<double> postXhist(100, -maxOfAll, maxOfAll);
  utils::SimpleHistogram<double> postYhist(100, -maxOfAll, maxOfAll);

  utils::SimpleHistogram<double> preParamhist(30, -MAX_ERR_VAL, MAX_ERR_VAL);
  utils::SimpleHistogram<double> postParamhist(300, -maxOfJointParamsAll,
                                               maxOfJointParamsAll);

  for (const auto &elem : jointErrList) {
    std::vector<double> newElem;
    for (auto &i : elem) {
      float val = i / TO_RAD;
      if (val > 0.01 || val < -0.01) {
        newElem.push_back(val);
      }
    }
    preParamhist.update(newElem);
  }
  // Update each histogram
  for (auto &elem : calibResidualList) {
    preXhist.update(elem.preX);
    preYhist.update(elem.preY);
    postXhist.update(elem.postX);
    postYhist.update(elem.postY);

    // For the moment, all joints are bundled together
    // TODO seperate this
    for (auto &jointErr : elem.jointResiduals) {
      std::vector<double> newElem;
      for (auto &i : jointErr) {
        if (i > __FLT_EPSILON__ && i < -__FLT_EPSILON__) {
          newElem.push_back(i);
        }
      }
      postParamhist.update(jointErr);
    }
  }

  std::cout << "Finishing; Bad cases: "
            << badCases.load() * 100 / (float)JOINT_ERR_LST_COUNT << "%"
            << std::endl;

  /*
   * Print the statistics & Write into files
   * TODO fix this in a better way. Maybe take OpenCV's method?
   */

  std::pair<utils::SimpleHistogram<double> &, std::string>
      originalResidualDumpX(preXhist, "/tmp/originalResidualX");
  std::pair<utils::SimpleHistogram<double> &, std::string>
      calibratedResidualDumpX(postXhist, "/tmp/calibratedResidualX");
  std::pair<utils::SimpleHistogram<double> &, std::string>
      originalResidualDumpY(preYhist, "/tmp/originalResidualY");
  std::pair<utils::SimpleHistogram<double> &, std::string>
      calibratedResidualDumpY(postYhist, "/tmp/calibratedResidualY");

  std::pair<utils::SimpleHistogram<double> &, std::string>
      calibratedJointParamResidualDumpY(postParamhist,
                                        "/tmp/calibratedJointResidual");

  std::pair<utils::SimpleHistogram<double> &, std::string> unCalibJointParams(
      preParamhist, "/tmp/unCalibratedJointResidual");

  for (auto elem : {originalResidualDumpX, originalResidualDumpY,
                    calibratedResidualDumpX, calibratedResidualDumpY,
                    calibratedJointParamResidualDumpY, unCalibJointParams}) {
    std::cout << "Start dump to : " << elem.second << std::endl;
    std::pair<double, double> bounds = elem.first.getPercentileBounds(0.25);
    std::cout << "75% Of Population in: " << bounds.first << " "
              << bounds.second << std::endl;
    bounds = elem.first.getPercentileBounds(0.1);
    std::cout << "90% Of Population in: " << bounds.first << " "
              << bounds.second << std::endl;
    bounds = elem.first.getPercentileBounds(0.05);
    std::cout << "95% Of Population in: " << bounds.first << " "
              << bounds.second << std::endl;
    bounds = elem.first.getPercentileBounds(0.01);
    std::cout << "99% Of Population in: " << bounds.first << " "
              << bounds.second << std::endl;
    bounds = elem.first.getPercentileBounds(0.001);
    std::cout << "99.9% Of Population in: : " << bounds.first << " "
              << bounds.second << std::endl;

    // Open files, stream and close
    std::fstream file(elem.second, std::ios::out);
    file << elem.first;
    file.close();
  }

  return 0;
}
