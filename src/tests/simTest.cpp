#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <iterator>
#include <sstream>
#include <thread>
#include <tuple>

#include <Data/CameraMatrix.hpp>
#include <Modules/Configuration/Configuration.h>
#include <Modules/Configuration/UnixSocketConfig.hpp>
#include <Modules/NaoProvider.h>
#include <Modules/Poses.h>

#include <cxxopts/include/cxxopts.hpp>

#define DEBUG_JOINT_AND_SENS 0
#define DEBUG_SIM_TEST 0
#define DO_COMMIT 1
#define WRITE_PARALLEL 1
#define DEBUG_FILE_COMMIT 1
#define DEBUG_IN_THREAD 1
#define ENABLE_PROGRESS 1

#include "JointCalibSolver.hpp"
#include "MiniConfigHandle.hpp"
#include "NaoPoseInfo.hpp"
#include "TUHHMin.hpp"
#include "constants.hpp"
#include "utils.hpp"

const unsigned int MAX_THREADS = std::thread::hardware_concurrency();

template <class T> using Residual = std::vector<T>;

enum CalibStatus {
  FAIL_LOCAL_MINIMA,
  FAIL_NO_CONVERGE,
  FAIL_NUMERICAL,
  FAIL_NO_CAPTURES,
  SUCCESS,
  MAX_STATUS_COUNT
};

using CalibStatusStatistics = std::array<size_t, CalibStatus::MAX_STATUS_COUNT>;

template <typename T> struct CalibEvalResiduals {
  Residual<T> preX;
  Residual<T> preY;
  Residual<T> postX;
  Residual<T> postY;

  std::array<Residual<T>, JointCalibSolvers::COMPACT_JOINT_CALIB_PARAM_COUNT>
      jointResiduals;

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
    for (size_t i = 0; i < JointCalibSolvers::COMPACT_JOINT_CALIB_PARAM_COUNT;
         ++i) {
      auto &srcjoint = src.jointResiduals[i];
      auto &dstjoint = dest.jointResiduals[i];
      dstjoint.insert(dstjoint.end(), std::make_move_iterator(srcjoint.begin()),
                      std::make_move_iterator(srcjoint.end()));
    }
    return dest;
  }
};

struct JointErrorEval {

  const NaoJointAndSensorModelConfig cfg;
  const poseAndRawAngleListT poseList;
  const SUPPORT_FOOT supFoot;
  const float jointCalibQualityTol;
  const float reprojErrTolPercent;
  const bool stochasticFix;
  const bool enablePixelNoise;
  const bool enableJointNoise;

  /// Random generator
  std::default_random_engine generator; // random gen
  std::normal_distribution<float> pixelNoiseDistribution;
  std::normal_distribution<double> jointNoiseDistribution;

  JointErrorEval(const NaoJointAndSensorModelConfig cfg,
                 const poseAndRawAngleListT poseList,
                 const SUPPORT_FOOT supFoot, const float jointCalibQualityTol,
                 const float reprojErrTolPercent, const float pixelNoiseStdDev,
                 const float jointNoiseStdDev, const bool stochasticFix,
                 const bool enablePixelNoise, const bool enableJointNoise)
      : cfg(cfg), poseList(poseList), supFoot(supFoot),
        jointCalibQualityTol(jointCalibQualityTol),
        reprojErrTolPercent(reprojErrTolPercent), stochasticFix(stochasticFix),
        enablePixelNoise(enablePixelNoise), enableJointNoise(enableJointNoise),
        generator(), pixelNoiseDistribution(0.0f, pixelNoiseStdDev),
        jointNoiseDistribution(0.0, static_cast<double>(jointNoiseStdDev)) {}

  /**
   * @brief evalJointErrorSet Evaluate calibration capability for an error
   * config.
   * THIS IS NOT THREAD SAFE
   * @param naoModel As mentioned in threadedFcn..
   * @param poseList
   * @param inducedErrorStdVec Joint error configuration
   * @return pre-post residuals residuals and more infos.
   */
  std::tuple<JointCalibSolvers::JointCalibResult, Eigen::VectorXf,
             Eigen::VectorXf, Eigen::VectorXf>
  evalJointErrorSet(const std::vector<Camera> camNames,
                    const rawPoseT inducedErrorStdVec,
                    CalibStatus &successStatus) {

#if DEBUG_SIM_TEST
    std::stringstream logStream;
    logStream << "BEGIN\n";
#endif
    // Create result object
    JointCalibSolvers::JointCalibResult result;

    // Map the error vector into an Eigen vector
    Eigen::VectorXf inducedErrorEigCompact;
    if (!JointCalibSolvers::rawPoseToJointCalibParams(inducedErrorStdVec,
                                                      inducedErrorEigCompact)) {
    }

    // Initialize residual of joint calibration (this isn't squared*)
    Eigen::VectorXf jointCalibResidual = inducedErrorEigCompact;

    // Make a copy of nao sensor model
    auto naoJointSensorModel = NaoJointAndSensorModel(cfg);
    // set the error state

    // List of captures
    JointCalibSolvers::CaptureList frameCaptures;

    // Capture per pose.
    size_t iter = 0;
    for (const auto &poseAndAngles : poseList) {
      naoJointSensorModel.setCalibValues(inducedErrorStdVec);
      if (enableJointNoise) {
        auto poseAngles(poseAndAngles.angles);
        for (auto &elem : poseAngles) {
          elem += static_cast<float>(jointNoiseDistribution(generator) *
                                     TO_RAD_DBL);
        }
        naoJointSensorModel.setPose(poseAngles, poseAndAngles.pose.supportFoot);
      } else {
        naoJointSensorModel.setPose(poseAndAngles.angles,
                                    poseAndAngles.pose.supportFoot);
      }
      for (const auto &camName : camNames) {
        bool proceed = false;
        // will be relative to support foot, easier to manage
        const auto groundGrid =
            naoJointSensorModel.getGroundGrid(camName, proceed);
        if (proceed) {
          // world points, pixel points.
          // TODO make world points handle 3d also
          auto correspondances =
              NaoSensorDataProvider::getFilteredCorrespondancePairs(
                  groundGrid,
                  naoJointSensorModel.robotToPixelMulti(camName, groundGrid));

          // add measurement noise
          if (enablePixelNoise) {
            for (auto &elem : correspondances) {
              elem.second.x() +=
                  static_cast<float>(pixelNoiseDistribution(generator));
              elem.second.y() +=
                  static_cast<float>(pixelNoiseDistribution(generator));
            }
          }
          if (correspondances.size() > 0) {
            frameCaptures.emplace_back(correspondances, poseAndAngles.angles,
                                       camName, poseAndAngles.pose.supportFoot);
          } else {
#if DEBUG_SIM_TEST
            //          std::lock_guard<std::mutex> lg(utils::mtx_cout_);
            logStream << "No suitable ground points " << iter << " "
                      << (camName == Camera::TOP ? "TOP" : "BOT") << std::endl;
#endif
          }
        } else {
#if DEBUG_SIM_TEST
          //        std::lock_guard<std::mutex> lg(utils::mtx_cout_);
          logStream << "Obtaining ground grid failed, gridCount: "
                    << groundGrid.size() << " pose:" << iter << " "
                    << " cam:" << (camName == Camera::TOP ? "TOP" : "BOT")
                    << std::endl;
#endif
        }
      }
      iter++;
    }

    // If no frames are captured, consider this as a failure
    if (frameCaptures.size() <= 0) {
      successStatus = CalibStatus::FAIL_NO_CAPTURES;
#if DEBUG_SIM_TEST
      //    std::lock_guard<std::mutex> lg(utils::mtx_cout_);
      logStream << "No suitable frame captures !!\n";
      logStream << "END";
      std::lock_guard<std::mutex> lg(utils::mtx_cout_);
      std::cout << logStream.str() << std::endl;
#endif
      return {result, jointCalibResidual, Eigen::VectorXf(0),
              Eigen::VectorXf(0)};
    } else {
#if DEBUG_SIM_TEST
      logStream << "Framecaptures: " << frameCaptures.size() << std::endl;
#endif
    }

    /*
     * Mini eval of cost fcn to get an idea of error
     */
    auto functor = JointCalibSolvers::JointCalibrator(frameCaptures, cfg);
    Eigen::NumericalDiff<JointCalibSolvers::JointCalibrator, Eigen::Central>
        calibrator(functor);
    //  auto calibrator = JointCalibSolvers::JointCalibrator(frameCaptures,
    //  cfg);

    Eigen::VectorXf errorVec(calibrator.values());
    Eigen::VectorXf finalErrorVec(calibrator.values());

    // Setting zero is very important.
    Eigen::VectorXf calibratedParams(static_cast<Eigen::Index>(
        JointCalibSolvers::COMPACT_JOINT_CALIB_PARAM_COUNT));
    calibratedParams.setZero();

    calibrator(calibratedParams, errorVec);

    /*
     * Run Lev-Mar to optimize
     */
    auto runLevMar = [&calibrator](Eigen::VectorXf &params) {

      Eigen::LevenbergMarquardt<
          Eigen::NumericalDiff<JointCalibSolvers::JointCalibrator,
                               Eigen::Central>,
          float>
          lm(calibrator);
      int status = lm.minimize(params);
      return status;
    };

    /// Run the Levenberg-Marquardt solver
    int status = runLevMar(calibratedParams);

    //  Get reprojection error
    calibrator(calibratedParams, finalErrorVec);

    // Just to note if the loop was done and broken with success
    bool loopedAndBroken = false;
    // Do stochastic fixing?
    if (stochasticFix && status != 4) {
      // This will hold best params in case looping is needed
      Eigen::VectorXf oldParams(static_cast<Eigen::Index>(
          JointCalibSolvers::COMPACT_JOINT_CALIB_PARAM_COUNT));
      size_t count = 0; // counter for the while loop
      /// TODO make this distribution configurable
      std::uniform_real_distribution<float> distribution(-6.5, 6.5);

      /*
       * If status is 2 or 5, most likely we are at a local minima
       * So to give some push-start random start positions will be attempted
       */
      const size_t legsInitialIdx =
          (supFoot == SUPPORT_FOOT::SF_DOUBLE ||
           supFoot == SUPPORT_FOOT::SF_LEFT)
              ? 2
              : 2 + JointCalibSolvers::LEG_JOINT_COUNT;
      const size_t legsEndIdx =
          (supFoot == SUPPORT_FOOT::SF_DOUBLE ||
           supFoot == SUPPORT_FOOT::SF_RIGHT)
              ? JointCalibSolvers::COMPACT_JOINT_CALIB_PARAM_COUNT
              : 2 + JointCalibSolvers::LEG_JOINT_COUNT;
      while ((status == 2 || status == 5) && count < 100) {
        // save state
        oldParams = calibratedParams;
        // get cost
        float finalCost = finalErrorVec.squaredNorm();

        /// Populate the starting point with random values

        calibratedParams(JOINTS::JOINT::HEAD_PITCH) =
            std::min(0.0f, static_cast<float>(distribution(generator)) / 2.0f *
                               TO_RAD_FLT);
        calibratedParams(JOINTS::JOINT::HEAD_YAW) =
            distribution(generator) * TO_RAD_FLT;

        /// We are now using the compact form

        for (size_t i = legsInitialIdx; i < legsEndIdx; i++) {
          calibratedParams(static_cast<Eigen::Index>(i)) =
              distribution(generator) * TO_RAD_FLT;
        }
        /// Run Lev-Mar again
        status = runLevMar(calibratedParams);
        // Get the new cost
        calibrator(calibratedParams, finalErrorVec);

        // Check status and continue accordingly
        if (status == 4 && finalCost > finalErrorVec.squaredNorm()) {
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
#if DEBUG_SIM_TEST
      {
        //      std::lock_guard<std::mutex> lg(utils::mtx_cout_);
        logStream << "i " << count << std::endl;
      }
#endif
    }

    /// Run Lev-Mar again
    //  status = runLevMar(calibratedParams);

    /*
     * END lev-mar
     */

    jointCalibResidual -= calibratedParams; // Joint residual

    // Get min-max info for error checking
    const auto minCoeff = finalErrorVec.minCoeff();
    const auto maxCoeff = finalErrorVec.maxCoeff();
    //  const auto finalErrAbsMax = std::max(std::abs(minCoeff),
    //  std::abs(maxCoeff));
    const auto finalErrorAvg = finalErrorVec.mean();
    const auto errorNorm = finalErrorVec.norm();
    //  const auto errorAvg = finalErrorVec.norm() / finalErrorVec.size();
    const auto jointCalibResAbsMax =
        std::max(std::abs(jointCalibResidual.maxCoeff()),
                 std::abs(jointCalibResidual.minCoeff()));
    const auto imageSize = naoJointSensorModel.getImSize();

    // Determine success or failure
    if (finalErrorAvg <= imageSize.minCoeff() * reprojErrTolPercent &&
        errorNorm <= errorVec.norm() &&
        jointCalibResAbsMax > jointCalibQualityTol) {
      successStatus = CalibStatus::FAIL_LOCAL_MINIMA;
    } else if (std::isnan(minCoeff) || std::isnan(maxCoeff)) {
      successStatus = CalibStatus::FAIL_NUMERICAL;
    } else if (finalErrorAvg > imageSize.minCoeff() * reprojErrTolPercent ||
               errorNorm > errorVec.norm()) {
      successStatus = CalibStatus::FAIL_NO_CONVERGE;
    } else {
      successStatus = CalibStatus::SUCCESS;
    }

//  if (successStatus != CalibStatus::SUCCESS) {
//  {
//    auto func = JointCalibSolvers::JointCalibrator(frameCaptures, cfg);
//    Eigen::NumericalDiff<JointCalibSolvers::JointCalibrator,
//                         Eigen::NumericalDiffMode::Central>
//        calib(func);

//    Eigen::VectorXf fakeParams(calibratedParams.size());
//    fakeParams.setZero();
//    Eigen::VectorXf temp(finalErrorVec.size());
//    temp.setZero();
//    calib(fakeParams, temp);
//    JointCalibSolvers::JointCalibrator::JacobianType mat(calib.values(),
//                                                         calib.inputs());
//    calib.df(fakeParams, mat);

//    size_t currentlyPrintedVals = 0;
//    for (size_t i = 0; i < func.captureList.size(); i++) {
//      const auto &cap = func.captureList[i];
//      const auto capCount = cap.capturedCorrespondances.size() * 2;
//      std::stringstream ss;
//      std::cout << (cap.camName == Camera::TOP ? "T" : "B")
//                << " n: " << capCount << " j: ";
//      for (const auto &j : cap.pose) {
//        ss << j << " ";
//      }
//      std::cout << ss.str() << std::endl;
//      ss << "\n\t | "
//         << temp.segment(static_cast<Eigen::Index>(currentlyPrintedVals),
//                         capCount)
//                .transpose()
//         << std::endl;
//      std::hash<std::string> hash_fn;
//      std::cout << "hashMain: " << hash_fn(ss.str());

//      ss.clear();
//      ss << mat.block(static_cast<Eigen::Index>(currentlyPrintedVals), 0,
//                      capCount, calib.inputs());
//      std::cout << " hashJac: " << hash_fn(ss.str()) << std::endl;
//      //      std::cout << mat.transpose() << std::endl;
//      currentlyPrintedVals += capCount;
//    }
//  std::cout << "\n " << successStatus << " Done\n\n\n" << std::endl;
//  }

// if failed OR if loop broken BUT calibration is bad
/*
 * NOTE: Local minima if reprojection errr is small but jointResidual isn't
 * low!
 */
#if DEBUG_SIM_TEST
    if (successStatus != CalibStatus::SUCCESS ||
        (stochasticFix && loopedAndBroken &&
         jointCalibResAbsMax > jointCalibQualityTol)) {

      //    std::lock_guard<std::mutex> lg(utils::mtx_cout_);
      logStream << "errAbsMax: "
                << std::max(std::abs(minCoeff), std::abs(maxCoeff)) << " "
                << " errorNorm: " << jointCalibResAbsMax
                << " errorAvg: " << jointCalibResAbsMax
                << " jointCalibResAbsMax: " << jointCalibResAbsMax / TO_RAD
                << " Status: " << status << std::endl;
    }
#endif
    // Update the result object
    result.status = status;
    result.jointParams.resize(JOINTS::JOINT::JOINTS_MAX);
    if (!JointCalibSolvers::jointCalibParamsToRawPose(calibratedParams,
                                                      result.jointParams)) {
    }
    result.reprojectionErrorNorm = errorNorm;
    result.reprojectionErrorNorm = finalErrorAvg;
#if DEBUG_SIM_TEST
    logStream << "END";
    std::lock_guard<std::mutex> lg(utils::mtx_cout_);
    std::cout << logStream.str() << std::endl;
#endif
    return {result, jointCalibResidual, errorVec, finalErrorVec};
  }
};

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
CalibStatusStatistics
threadedFcn(CalibEvalResiduals<double> &residuals,
            const poseAndRawAngleListT poseList, const SUPPORT_FOOT supFoot,
            const std::vector<Camera> camNames,
            const NaoJointAndSensorModelConfig cfg,
            const std::vector<rawPoseT>::iterator jointErrItrBegin,
            const std::vector<rawPoseT>::iterator jointErrItrEnd,
            const float jointCalibQualityTol, const float reprojErrTolPercent,
            const float pixelNoiseStdDev, const float jointNoiseStdDev,
            bool stochasticFix, const bool enablePixelNoise,
            const bool enableJointNoise) {

  CalibStatusStatistics calibStatusStatistics = {};

  //  JointErrorEval(const NaoJointAndSensorModelConfig cfg,
  //                 const poseAndRawAngleListT poseList,
  //                 const SUPPORT_FOOT supFoot, const float
  //                 jointCalibQualityTol,
  //                 const float reprojErrTolPercent, const float
  //                 pixelNoiseStdDev,
  //                 const float jointNoiseStdDev, const bool stochasticFix,
  //                 const bool enablePixelNoise, const bool enableJointNoise)

  JointErrorEval jointErrorEvalStruct(
      cfg, poseList, supFoot, jointCalibQualityTol, reprojErrTolPercent,
      pixelNoiseStdDev, jointNoiseStdDev, stochasticFix, enablePixelNoise,
      enableJointNoise);

  for (auto errSetIter = jointErrItrBegin; errSetIter != jointErrItrEnd;
       ++errSetIter) {

    auto &errorSet = *errSetIter;

    CalibStatus calibStatus; // change this to more detailed output

    JointCalibSolvers::JointCalibResult result;
    Eigen::VectorXf preResiduals;
    Eigen::VectorXf postResiduals;
    Eigen::VectorXf jointCalibResiduals;
    // Call evaluator for this error config.
    std::tie(result, jointCalibResiduals, preResiduals, postResiduals) =
        jointErrorEvalStruct.evalJointErrorSet(camNames, errorSet, calibStatus);

    calibStatusStatistics[calibStatus]++;

    auto prePostResSize = static_cast<size_t>(preResiduals.size());
    auto curPreResSize = residuals.preX.size();

    // reserve vector memory to speed up the process
    residuals.preX.reserve(prePostResSize + curPreResSize);
    residuals.postX.reserve(prePostResSize + curPreResSize);
    residuals.preY.reserve(prePostResSize + curPreResSize);
    residuals.postY.reserve(prePostResSize + curPreResSize);

    for (long i = 0; i < preResiduals.size(); ++i) {
      if (i % 2 == 0) { // x -> even number
        residuals.preX.push_back(static_cast<double>(preResiduals(i)));
        residuals.postX.push_back(static_cast<double>(postResiduals(i)));
      } else { // odd  -> newline
        residuals.preY.push_back(static_cast<double>(preResiduals(i)));
        residuals.postY.push_back(static_cast<double>(postResiduals(i)));
      }
    }
    // Convert to degrees
    jointCalibResiduals /= TO_RAD_FLT;
    for (Eigen::Index i = 0;
         i < JointCalibSolvers::COMPACT_JOINT_CALIB_PARAM_COUNT; ++i) {
      // TODO FIX THIS
      //      residuals.jointResiduals[static_cast<size_t>(i)].push_back(
      //          jointCalibResiduals(i));
      residuals.jointResiduals[static_cast<size_t>(i)].push_back(
          static_cast<double>(jointCalibResiduals(i)));
    }
  }
  return calibStatusStatistics;
}

/**
 * @brief main
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char *argv[]) {

  cxxopts::Options options("PoseFilter - Filter and link poses to hopefully "
                           "give the best pose set");
  options.add_options()("f,file-prefix", "I/O File prefix",
                        cxxopts::value<std::string>())(
      "e,errorConfigs", "set of error cases",
      cxxopts::value<std::string>()->default_value("./l_1000_jointErrors.txt"))(
      "n,n-poses", "Poses to calibrate",
      cxxopts::value<size_t>()->default_value("10"))(
      "m,mirror", "Mirror the poses",
      cxxopts::value<bool>()->default_value("false"))(
      "c,confRoot", "Conf path",
      cxxopts::value<std::string>()->default_value("../../nao/home/"))(
      "s,stochastic", "Stochastic mode?",
      cxxopts::value<bool>()->default_value("false"))(
      "h,histogram", "show histogram",
      cxxopts::value<bool>()->default_value("false"))(
      "j,jointNoise", "apply joint noise",
      cxxopts::value<bool>()->default_value("false"))(
      "p,pixelNoise", "apply pixel noise",
      cxxopts::value<bool>()->default_value("false"));

  auto result = options.parse(argc, argv);

  const std::string inFileName = result["file-prefix"].as<std::string>();
  const std::string errConfigFileName =
      result["errorConfigs"].as<std::string>();
  const size_t MAX_POSES_TO_CALIB = result["n-poses"].as<size_t>();
  // Default is false
  const bool mirrorPose = result["mirror"].as<bool>();
  const std::string confRoot = result["confRoot"].as<std::string>();
  // Default is false
  const bool stochasticFix = result["stochastic"].as<bool>();
  const bool showHistogram = result["histogram"].as<bool>();

  const bool enablePixelNoise = result["pixelNoise"].as<bool>();
  const bool enableJointNoise = result["jointNoise"].as<bool>();

  std::fstream inFile(inFileName);
  if (!inFile.is_open()) {
    std::cerr << "Input file cannot be opened, exiting.." << std::endl;
    return 1;
  }
  std::fstream errorConfFile(errConfigFileName);
  if (!errorConfFile.is_open()) {
    std::cerr << "error config file cannot be opened, exiting.." << std::endl;
    return 1;
  }

  /// Get config values.
  Uni::Value confValue;
  if (!MiniConfigHandle::mountFile("configuration/simGridEvalConf.json",
                                   confValue)) {
    std::cout << "couldn't open the conf file" << std::endl;
    exit(1);
  }
  /// observerModel config (to get grpound grid info, etc)
  Uni::Value obsModelConfig;
  if (!MiniConfigHandle::mountFile("configuration/cameraObsModelConf.json",
                                   obsModelConfig)) {
    std::cout << "couldn't open the conf file" << std::endl;
    exit(1);
  }

  const float MIN_ERR_VAL =
      static_cast<float>(confValue["errRangeMinMax"].at(0).asDouble());
  const float MAX_ERR_VAL =
      static_cast<float>(confValue["errRangeMinMax"].at(1).asDouble());
  /// Sample Size
  const size_t JOINT_ERR_LST_DESIRED_COUNT =
      static_cast<size_t>(confValue["testErrorCount"].asInt64());

  const double JOINT_CALIB_QUALITY_TOL_DEG =
      confValue["jointCalibQualityTol"].asDouble();
  const float JOINT_CALIB_QUALITY_TOL_RAD =
      static_cast<float>(JOINT_CALIB_QUALITY_TOL_DEG * TO_RAD_DBL);
  const float REPROJ_ERR_TOL_PERCENT =
      static_cast<float>(confValue["reprojErrTolPercent"].asDouble());

  const float PIXEL_NOISE_STD_DEV =
      static_cast<float>(confValue["pixelNoiseStdDev"].asDouble());

  const float JOINT_NOISE_STD_DEV =
      static_cast<float>(confValue["jointNoiseStdDev"].asDouble());
  /*
   * Print input params
   */
  std::cout << "Errors to try:\t" << JOINT_ERR_LST_DESIRED_COUNT
            << "\nPoses to try:\t" << MAX_POSES_TO_CALIB
            << "\nMinMax Error val:\t" << MIN_ERR_VAL << ", " << MAX_ERR_VAL
            << "\nMirror Poses:\t" << mirrorPose << "\nStochastic Fix:\t"
            << stochasticFix << "\nenablePixelNoise:\t" << enablePixelNoise
            << "\nenableJointNoise:\t" << enableJointNoise
            << "\nJoint Calib Tol (deg) :\t" << JOINT_CALIB_QUALITY_TOL_DEG
            << std::endl;

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

  Vector2i imSize;
  size_t maxGridPointsPerSide;
  float gridSpacing;
  obsModelConfig["imSize"] >> imSize;
  obsModelConfig["gridSpacing"] >> gridSpacing;
  maxGridPointsPerSide = static_cast<size_t>(std::ceil(
      static_cast<float>(obsModelConfig["gridSizeLength"].asDouble()) /
      gridSpacing));

  CameraMatrix camMat;
  camMat.fc = fc;
  camMat.fc.x() *= imSize.x();
  camMat.fc.y() *= imSize.y();
  camMat.cc = cc;
  camMat.cc.x() *= imSize.x();
  camMat.cc.y() *= imSize.y();
  camMat.fov = fov;

  const NaoJointAndSensorModelConfig cfg = {imSize, fc, cc, fov, 50, 0.05f};

  std::cout << imSize.transpose() << " spc: " << gridSpacing
            << " maxGridPointsPerSize: " << maxGridPointsPerSide << std::endl;
  /*
   * Read poses
   */
  SUPPORT_FOOT supFeet = SUPPORT_FOOT::SF_NONE;

  poseAndRawAngleListT poseList;
  poseList.reserve(MAX_POSES_TO_CALIB);
  poseAndRawAngleT poseAndAngles;

  for (size_t i = 0; i < MAX_POSES_TO_CALIB &&
                     utils::JointsAndPosesStream::getNextPoseAndRawAngles(
                         inFile, poseAndAngles);
       ++i) {
    if (poseAndAngles.pose.supportFoot == SUPPORT_FOOT::SF_NONE) {
      --i;
      continue;
    }
    // Mirror this pose. Nice in testing ;)
    if (mirrorPose) {
      poseAndAngles.mirror();
    }
    // if already double, skip
    if (supFeet != SUPPORT_FOOT::SF_DOUBLE) {
      // if none, first try..
      if (supFeet == SUPPORT_FOOT::SF_NONE) {
        supFeet = poseAndAngles.pose.supportFoot;
      } else if (supFeet != poseAndAngles.pose.supportFoot) { // if differs but
                                                              // not none, then
                                                              // double it is ;)
        supFeet = SUPPORT_FOOT::SF_DOUBLE;
      }
    }
    poseList.push_back(poseAndAngles);
  }
  std::cout << " reading of poses done, SF: " << supFeet << std::endl;

  //  std::srand(std::time(0));
  //  std::random_shuffle(poseList.begin(), poseList.end());
  for (const auto &elem : poseList) {
    std::cout << "Pose " << elem.pose.id << std::endl;
  }

  /*
   * read the data
   */

  std::vector<rawPoseT> jointErrList;
  {
    std::string str;
    std::vector<float> vec(JOINTS::JOINTS_MAX);

    for (size_t iter = 0;
         iter < JOINT_ERR_LST_DESIRED_COUNT && errorConfFile.good() &&
         std::getline(errorConfFile, str);
         ++iter) {
      auto line = std::stringstream(str);
      for (size_t i = 0; i < JOINTS::JOINTS_MAX; i++) {
        line >> vec[i];
      }
      if (mirrorPose) {
        PoseUtils::mirrorJoints(vec);
      }
      jointErrList.push_back(vec);
    }
  }
  const size_t JOINT_ERR_LST_COUNT = jointErrList.size();
  std::cout << jointErrList.size() << "error lists to try" << std::endl;

  /*
   * Prepare for multithreading and start calibration
   */
  std::cout << "Starting calibrations" << std::endl;

  // VecOfVec of CalibEvalResiduals - they contain image and joint residual
  // info
  // + others
  std::vector<CalibEvalResiduals<double>> calibResidualList(MAX_THREADS);
  // This will contain all above at the end. (move-join)
  CalibEvalResiduals<double> finalResidualSet;

  // vector holding the std::threads
  std::vector<std::future<CalibStatusStatistics>> futureList(MAX_THREADS);

  // Setup offsets for iterators
  size_t threadingOffsets = JOINT_ERR_LST_COUNT / MAX_THREADS;

  // cameras to evaluate
  std::vector<Camera> camNames = {Camera::BOTTOM, Camera::TOP};

  // Start the threads
  for (size_t i = 0; i < MAX_THREADS; ++i) {

    std::vector<rawPoseT>::iterator begin = jointErrList.begin();
    std::advance(begin, (threadingOffsets * i));
    std::vector<rawPoseT>::iterator end = jointErrList.begin();
    if (i + 1 >= MAX_THREADS) {
      end = jointErrList.end();
    } else {
      std::advance(end, (threadingOffsets * (i + 1)));
    }

    std::cout << "starting thread: " << i << std::endl;
    //    threadedFcn(CalibEvalResiduals<double> &residuals,
    //                const poseAndRawAngleListT poseList, const SUPPORT_FOOT
    //                supFoot,
    //                const std::vector<Camera> camNames,
    //                const NaoJointAndSensorModelConfig cfg,
    //                const std::vector<rawPoseT>::iterator jointErrItrBegin,
    //                const std::vector<rawPoseT>::iterator jointErrItrEnd,
    //                const float jointCalibQualityTol, const float
    //                reprojErrTolPercent,
    //                const float pixelNoiseStdDev, const float
    //                jointNoiseStdDev,
    //                bool stochasticFix, const bool enablePixelNoise,
    //                const bool enableJointNoise)
    futureList[i] = std::async(
        std::launch::async, &threadedFcn, std::ref(calibResidualList[i]),
        poseList, supFeet, camNames, cfg, begin, end,
        JOINT_CALIB_QUALITY_TOL_RAD, REPROJ_ERR_TOL_PERCENT,
        PIXEL_NOISE_STD_DEV, JOINT_NOISE_STD_DEV, stochasticFix,
        enablePixelNoise, enableJointNoise);
  }

  /*
   * Finish calibration - join threads
   * Also join all residuals into finalResidualSet
   */
  CalibStatusStatistics stats = {};
  for (size_t i = 0; i < MAX_THREADS; ++i) {
    auto &future = futureList[i];
    auto &residual = calibResidualList[i];
    auto temp = future.get();
    for (size_t j = 0; j < stats.size(); ++j) {
      stats[j] += temp[j];
    }
    std::cout << "Going to join" << i << std::endl;
    finalResidualSet.joinEvalResiduals(residual, finalResidualSet);
  }
  std::cout << "Processing done" << std::endl;

  /*
   * Print stats for each joint
   */
  {
    auto printStats = [&JOINT_CALIB_QUALITY_TOL_DEG,
                       &JOINT_ERR_LST_COUNT](Residual<double> &resVec) {
      if (resVec.size() > JOINT_ERR_LST_COUNT) {
        std::cerr << "Something is wrong, resVec.size() > JOINT_ERR_LST_COUNT"
                  << std::endl;
        throw("Something is wrong, resVec.size() > JOINT_ERR_LST_COUNT");
      }

      //      auto minMax = std::minmax_element(resVec.begin(), resVec.end());
      //      utils::SimpleHistogram<double> tempHist(300, *minMax.first - 1,
      //                                              *minMax.second + 1);
      double avg =
          std::accumulate(resVec.begin(), resVec.end(), 0.0) / resVec.size();
      //      tempHist.update(resVec);
      size_t badCount = 0;
      for (auto &elem : resVec) {
        if (std::fabs(elem) > JOINT_CALIB_QUALITY_TOL_DEG) {
          badCount++;
        }
      }
      std::cout << " \tbad: "
                << badCount * 100 / static_cast<double>(JOINT_ERR_LST_COUNT)
                << "% \tavg: " << avg << "\n";
    };

    std::cout << "\n";
    for (size_t i = 0; i < finalResidualSet.jointResiduals.size(); ++i) {
      std::cout << "Joint " << Sensor::JOINT_NAMES[Sensor::ALL_OBS_JOINTS[i]];
      printStats(finalResidualSet.jointResiduals[i]);
    }
    std::cout << std::endl;
  }

  /*
   * Post-processing -> Get min-max bounds
   */

  std::cout << "Initializing histograms\n";
  auto minMaxPostX = std::minmax_element(finalResidualSet.postX.begin(),
                                         finalResidualSet.postX.end());

  auto minMaxPostY = std::minmax_element(finalResidualSet.postY.begin(),
                                         finalResidualSet.postY.end());

  auto maxOfJointParamsAll = [&finalResidualSet] {
    auto max = 0.0;
    for (const auto &vec : finalResidualSet.jointResiduals) {
      auto v = std::minmax_element(vec.begin(), vec.end());
      max = std::max(max, std::max(std::abs(*v.first), std::abs(*v.second)));
    }
    return max;
  }();
  //  auto maxOfJointParamsAll =
  //      1.01 * std::max(std::abs(*minMaxJointParams.first),
  //                      std::abs(*minMaxJointParams.second));

  auto maxOfAll = std::max(std::abs(*minMaxPostX.first), *minMaxPostX.second);
  maxOfAll = 1.01 * std::max(std::max(std::abs(*minMaxPostY.first),
                                      *minMaxPostY.second),
                             maxOfAll);

  std::cout << "AbsMax Post Errors: " << maxOfAll
            << "\nAbsMax Joint Errors: " << maxOfJointParamsAll << std::endl;

  /*
   * Initialize Histograms
   */
  utils::SimpleHistogram<double> preXhist(1000, -300, 300);
  utils::SimpleHistogram<double> preYhist(1000, -300, 300);

  utils::SimpleHistogram<double> postXhist(100, -maxOfAll, maxOfAll);
  utils::SimpleHistogram<double> postYhist(100, -maxOfAll, maxOfAll);

  utils::SimpleHistogram<double> preParamhist(
      30, static_cast<double>(-MAX_ERR_VAL), static_cast<double>(MAX_ERR_VAL));
  utils::SimpleHistogram<double> postParamhist(300, -maxOfJointParamsAll,
                                               maxOfJointParamsAll);

  for (const auto &elem : jointErrList) {
    std::vector<double> newElem;
    for (auto &i : elem) {
      float val = i / TO_RAD_FLT;
      if (val > 0.01f || val < -0.01f) {
        newElem.push_back(static_cast<double>(val));
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
        if (i > __DBL_EPSILON__ || i < -__DBL_EPSILON__) {
          newElem.push_back(i);
        }
      }
      postParamhist.update(newElem);
    }
  }

  std::cout << "\nFinishing; Total Bad cases: "
            << (std::accumulate(stats.begin(), stats.end(),
                                static_cast<size_t>(0)) -
                stats[CalibStatus::SUCCESS]) *
                   100 / static_cast<float>(JOINT_ERR_LST_COUNT)
            << "%\n"
            << "FAIL_LOCAL_MINIMA: "
            << stats[CalibStatus::FAIL_LOCAL_MINIMA] * 100 /
                   static_cast<float>(JOINT_ERR_LST_COUNT)
            << "%\n"
            << "FAIL_NO_CONVERGE: "
            << stats[CalibStatus::FAIL_NO_CONVERGE] * 100 /
                   static_cast<float>(JOINT_ERR_LST_COUNT)
            << "%\n"
            << "FAIL_NUMERICAL: "
            << stats[CalibStatus::FAIL_NUMERICAL] * 100 /
                   static_cast<float>(JOINT_ERR_LST_COUNT)
            << "%\n"
            << "FAIL_NO_CAPTURES: "
            << stats[CalibStatus::FAIL_NO_CAPTURES] * 100 /
                   static_cast<float>(JOINT_ERR_LST_COUNT)
            << "%\n"
            << "SUCCESS: "
            << stats[CalibStatus::SUCCESS] * 100 /
                   static_cast<float>(JOINT_ERR_LST_COUNT)
            << "%\n"
            << std::endl;

  if (!showHistogram) {
    return 0;
  }
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
                    calibratedResidualDumpX, calibratedResidualDumpY}) {
    std::cout << "\nStart dump to : " << elem.second << "\n";
    std::pair<double, double> bounds = elem.first.getPercentileBounds(0.25);
    std::cout << "75% Of Population in: " << bounds.first << " "
              << bounds.second << "\n";
    bounds = elem.first.getPercentileBounds(0.1);
    std::cout << "90% Of Population in: " << bounds.first << " "
              << bounds.second << "\n";
    bounds = elem.first.getPercentileBounds(0.05);
    std::cout << "95% Of Population in: " << bounds.first << " "
              << bounds.second << "\n";
    bounds = elem.first.getPercentileBounds(0.01);
    std::cout << "99% Of Population in: " << bounds.first << " "
              << bounds.second << "\n";
    bounds = elem.first.getPercentileBounds(0.001);
    std::cout << "99.9% Of Population in: : " << bounds.first << " "
              << bounds.second << std::endl;

    // Open files, stream and close
    std::fstream file(elem.second, std::ios::out);
    file << elem.first;
    file.close();
  }

  /*
   * Put joint residuals seperately for real box plots and other stuff :P
   */
  for (auto elem : {unCalibJointParams, calibratedJointParamResidualDumpY}) {
    std::cout << "\nStart dump to : " << elem.second << "\n";
    std::pair<double, double> bounds = elem.first.getPercentileBounds(0.25);
    std::cout << "75% Of Population in: " << bounds.first << " "
              << bounds.second << "\n";
    bounds = elem.first.getPercentileBounds(0.1);
    std::cout << "90% Of Population in: " << bounds.first << " "
              << bounds.second << "\n";
    bounds = elem.first.getPercentileBounds(0.05);
    std::cout << "95% Of Population in: " << bounds.first << " "
              << bounds.second << "\n";
    bounds = elem.first.getPercentileBounds(0.01);
    std::cout << "99% Of Population in: " << bounds.first << " "
              << bounds.second << "\n";
    bounds = elem.first.getPercentileBounds(0.001);
    std::cout << "99.9% Of Population in: : " << bounds.first << " "
              << bounds.second << std::endl;

    // Open files, stream and close
    std::fstream file(elem.second, std::ios::out);
    {
      for (const auto &residualSet : finalResidualSet.jointResiduals) {
        for (auto p = residualSet.begin(); p != residualSet.end() - 1; ++p) {
          file << *p << ", ";
        }
        if (residualSet.size() > 0) {
          file << *residualSet.end() << std::endl;
        } else {
          file << std::endl;
        }
      }
    }
    file.close();
  }

  return 0;
}
