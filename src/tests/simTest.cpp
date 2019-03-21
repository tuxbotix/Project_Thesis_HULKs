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

#include <cxxopts/include/cxxopts.hpp>

#define DEBUG_JOINT_AND_SENS 0
#define DEBUG_SIM_TEST 0
#define DO_COMMIT 1
#define WRITE_PARALLEL 1
#define DEBUG_FILE_COMMIT 1
#define DEBUG_IN_THREAD 1
#define ENABLE_PROGRESS 1

#include "JointCalibEval.hpp"
#include "JointCalibSolver.hpp"
#include "MiniConfigHandle.hpp"
#include "NaoPoseInfo.hpp"
#include "TUHHMin.hpp"
#include "constants.hpp"
#include "utils.hpp"

const unsigned int MAX_THREADS = std::thread::hardware_concurrency();

namespace jCalib = JointCalibration;
namespace jCalibSolver = JointCalibSolvers;
namespace jCEval = JointCalibEval;
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
std::pair<jCalib::CalibStatusStatistics, std::vector<jCalib::JointCalibResult>>
threadedFcn(jCalib::CalibEvalResiduals<double> &residuals,
            const jCEval::CombinedPoseListT combinedPoseList,
            const jCEval::CombinedPoseListT combinedTestPoseList,
            const SUPPORT_FOOT supFoot, const NaoJointAndSensorModelConfig cfg,
            const std::vector<rawPoseT>::iterator jointErrItrBegin,
            const std::vector<rawPoseT>::iterator jointErrItrEnd,
            const float minJointErrVal, const float maxJointErrVal,
            const float jointCalibQualityTol, const float reprojErrTolPercent,
            const float pixelNoiseStdDev, const float jointNoiseStdDev,
            bool stochasticFix, const bool enablePixelNoise,
            const bool enableJointNoise) {

  jCalib::CalibStatusStatistics calibStatusStatistics = {};
  std::vector<jCalib::JointCalibResult> resultList;
  resultList.reserve(size_t(jointErrItrEnd - jointErrItrBegin));
  //  jCEval::JointErrorEval(const NaoJointAndSensorModelConfig cfg,
  //                 const poseAndRawAngleListT poseList,
  //                 const SUPPORT_FOOT supFoot, const float
  //                 jointCalibQualityTol,
  //                 const float reprojErrTolPercent, const float
  //                 pixelNoiseStdDev,
  //                 const float jointNoiseStdDev, const bool stochasticFix,
  //                 const bool enablePixelNoise, const bool enableJointNoise)

  jCEval::JointErrorEval jointErrorEvalStruct(
      cfg, combinedPoseList, combinedTestPoseList, supFoot, minJointErrVal,
      maxJointErrVal, jointCalibQualityTol, reprojErrTolPercent,
      pixelNoiseStdDev, jointNoiseStdDev, stochasticFix, enablePixelNoise,
      enableJointNoise);

  for (auto errSetIter = jointErrItrBegin; errSetIter != jointErrItrEnd;
       ++errSetIter) {

    auto &errorSet = *errSetIter;

    jCalib::JointCalibResult result;
    Eigen::VectorXf jointCalibResiduals;
    Eigen::VectorXf preTestResiduals;
    Eigen::VectorXf preCalibResiduals;
    Eigen::VectorXf postTestResiduals;
    Eigen::VectorXf postCalibResiduals;

    //    return {result,        jointCalibResidual,    reprojErrTestInitial,
    //            reprojErrTest, reprojErrCalibInitial, reprojErrCalib};
    // Call evaluator for this error cnfig.
    std::tie(result, jointCalibResiduals, preTestResiduals, postTestResiduals,
             preCalibResiduals, postCalibResiduals) =
        jointErrorEvalStruct.evalJointErrorSet(errorSet);

    resultList.push_back(result);
    calibStatusStatistics[result.status]++;

    auto prePostResSize = static_cast<size_t>(preTestResiduals.size());
    auto curPreResSize = residuals.preX.size();

    // reserve vector memory to speed up the process
    residuals.preX.reserve(prePostResSize + curPreResSize);
    residuals.postX.reserve(prePostResSize + curPreResSize);
    residuals.preY.reserve(prePostResSize + curPreResSize);
    residuals.postY.reserve(prePostResSize + curPreResSize);

    for (long i = 0; i < preTestResiduals.size(); ++i) {
      if (i % 2 == 0) { // x -> even number
        residuals.preX.push_back(static_cast<double>(preTestResiduals(i)));
        residuals.postX.push_back(static_cast<double>(postTestResiduals(i)));
      } else { // odd  -> newline
        residuals.preY.push_back(static_cast<double>(preTestResiduals(i)));
        residuals.postY.push_back(static_cast<double>(postTestResiduals(i)));
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
  return {calibStatusStatistics, resultList};
}

/**
 * @brief getCombinedPosesWROI
 * @param poseList
 * @param camNames
 * @param naoJointSensorModel
 * @param roiFileName
 * @return
 */
jCEval::CombinedPoseListT getCombinedPosesWROI(
    const poseAndRawAngleListT &poseList, const std::vector<Camera> &camNames,
    NaoJointAndSensorModel naoJointSensorModel, std::string roiFileName = "") {

  jCEval::CombinedPoseListT combinedPoseList;
  std::fstream roiFile(roiFileName, std::ios::out);
  bool writeToFile = false;
  if (!roiFile.is_open()) {
    std::cout << "file not specified or open fail;" << std::endl;
  } else {
    writeToFile = true;
  }

  for (const auto &elem : poseList) {
    naoJointSensorModel.setPose(elem.angles, elem.pose.supportFoot);
    for (const auto &cam : camNames) {
      std::array<Vector2f, 4> corners = {};
      // clockwise order
      Vector2f camCenterProjPoint;
      if (naoJointSensorModel.projectCameraFOVToGround(cam, corners,
                                                       camCenterProjPoint)) {
        if (writeToFile) {
          roiFile << elem.pose.id << " "
                  << (cam == Camera::TOP ? "top" : "bottom") << " ";

          for (const auto &corner : corners) {
            roiFile << corner.x() << " " << corner.y() << " ";
          }
          roiFile << camCenterProjPoint.norm() << "\n";
        }
        combinedPoseList.emplace_back(elem, cam);
      }
    }
  }
  if (writeToFile) {
    roiFile.flush();
    roiFile.close();
  }
  return combinedPoseList;
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
      "t,testFile", "testPoseFile", cxxopts::value<std::string>())(
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
  const std::string inTestFileName = result["testFile"].as<std::string>();
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
  std::fstream inTestFile(inTestFileName);
  if (!inTestFile.is_open()) {
    std::cerr << "Test file cannot be opened, exiting.." << std::endl;
    return 1;
  }
  std::fstream errorConfFile(errConfigFileName);
  if (!errorConfFile.is_open()) {
    std::cerr << "Config file cannot be opened, exiting.." << std::endl;
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
  const size_t testPoseCountMin =
      static_cast<size_t>(confValue["testPoseCountMin"].asInt64());
  const size_t testPoseCount =
      static_cast<size_t>(confValue["testPoseCount"].asInt64());
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

  for (;
       poseList.size() <= MAX_POSES_TO_CALIB &&
       JointsAndPosesStream::getNextPoseAndRawAngles(inFile, poseAndAngles);) {
    if (poseAndAngles.pose.supportFoot == SUPPORT_FOOT::SF_NONE) {
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
  if (supFeet == SUPPORT_FOOT::SF_NONE) {
    std::cerr << "No valid support feet found!! exiting..." << std::endl;
  }
  std::cout << " reading of poses done, SF: " << supFeet << std::endl;

  /// Add test poses
  poseAndRawAngleListT testPoseList;
  for (size_t i = 0;
       i < std::max(size_t(100), testPoseCount) &&
       testPoseList.size() <= testPoseCount &&
       JointsAndPosesStream::getNextPoseAndRawAngles(inTestFile, poseAndAngles);
       ++i) {
    /// TODO TEMP add support for double foot
    if (poseAndAngles.pose.supportFoot == SUPPORT_FOOT::SF_NONE ||
        poseAndAngles.pose.supportFoot != supFeet) {
      continue;
    }
    // Mirror this pose. Nice in testing ;)
    if (mirrorPose) {
      poseAndAngles.mirror();
    }
    testPoseList.push_back(poseAndAngles);
  }

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
  std::vector<jCalib::CalibEvalResiduals<double>> calibResidualList(
      MAX_THREADS);
  // This will contain all above at the end. (move-join)
  jCalib::CalibEvalResiduals<double> finalResidualSet;

  // vector holding the std::threads
  std::vector<std::future<std::pair<jCalib::CalibStatusStatistics,
                                    std::vector<jCalib::JointCalibResult>>>>
      futureList(MAX_THREADS);

  // Setup offsets for iterators
  size_t threadingOffsets = JOINT_ERR_LST_COUNT / MAX_THREADS;

  // cameras to evaluate
  std::vector<Camera> camNames = {Camera::BOTTOM, Camera::TOP};

  // PoseAndCamera list
  jCEval::CombinedPoseListT combinedPoseList;
  jCEval::CombinedPoseListT combinedTestPoseList;
  {
    auto naoJointSensorModel = NaoJointAndSensorModel(cfg);
    combinedPoseList = getCombinedPosesWROI(
        poseList, camNames, naoJointSensorModel, "/tmp/rois.txt");
    combinedTestPoseList = getCombinedPosesWROI(
        testPoseList, camNames, naoJointSensorModel, "/tmp/test_rois.txt");
  }

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
        combinedPoseList, combinedTestPoseList, supFeet, cfg, begin, end,
        MIN_ERR_VAL, MAX_ERR_VAL, JOINT_CALIB_QUALITY_TOL_RAD,
        REPROJ_ERR_TOL_PERCENT, PIXEL_NOISE_STD_DEV, JOINT_NOISE_STD_DEV,
        stochasticFix, enablePixelNoise, enableJointNoise);
  }

  /*
   * Finish calibration - join threads
   * Also join all residuals into finalResidualSet
   * + calibration results!!
   */
  jCalib::CalibStatusStatistics stats = {};
  std::vector<jCalib::JointCalibResult> resultList;
  for (size_t i = 0; i < MAX_THREADS; ++i) {
    auto &future = futureList[i];
    auto &residual = calibResidualList[i];
    auto temp = future.get();
    for (size_t j = 0; j < stats.size(); ++j) {
      stats[j] += temp.first[j];
    }
    // append do the result vector
    resultList.insert(resultList.end(),
                      std::make_move_iterator(temp.second.begin()),
                      std::make_move_iterator(temp.second.end()));
    std::cout << "Going to join" << i << std::endl;
    finalResidualSet.joinEvalResiduals(residual, finalResidualSet);
  }
  std::cout << "Processing done" << std::endl;

  /*
   * Print stats for each joint
   */
  {
    auto printStats = [&JOINT_CALIB_QUALITY_TOL_DEG,
                       &JOINT_ERR_LST_COUNT](jCalib::Residual<double> &resVec) {
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
                stats[jCalib::CalibStatus::SUCCESS]) *
                   100 / static_cast<float>(JOINT_ERR_LST_COUNT)
            << "%\n"
            << "FAIL_LOCAL_MINIMA: "
            << stats[jCalib::CalibStatus::FAIL_LOCAL_MINIMA] * 100 /
                   static_cast<float>(JOINT_ERR_LST_COUNT)
            << "%\n"
            << "FAIL_NO_CONVERGE: "
            << stats[jCalib::CalibStatus::FAIL_NO_CONVERGE] * 100 /
                   static_cast<float>(JOINT_ERR_LST_COUNT)
            << "%\n"
            << "FAIL_NUMERICAL: "
            << stats[jCalib::CalibStatus::FAIL_NUMERICAL] * 100 /
                   static_cast<float>(JOINT_ERR_LST_COUNT)
            << "%\n"
            << "FAIL_NO_CAPTURES: "
            << stats[jCalib::CalibStatus::FAIL_NO_CAPTURES] * 100 /
                   static_cast<float>(JOINT_ERR_LST_COUNT)
            << "%\n"
            << "FAIL_NO_TEST_CAPTURES: "
            << stats[jCalib::CalibStatus::FAIL_NO_TEST_CAPTURES] * 100 /
                   static_cast<float>(JOINT_ERR_LST_COUNT)
            << "%\n"
            << "SUCCESS: "
            << stats[jCalib::CalibStatus::SUCCESS] * 100 /
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
  {
    /// Begin: printing and saving RMS values
    // arr1[ <[vec2d] * 6, size_t> ]
    // [ rmstest minRmsTest maxRmsTest rmsCalib ... ]
    std::array<std::pair<std::array<Eigen::Vector2d, 6>, size_t>,
               jCalib::CalibStatus::MAX_STATUS_COUNT>
        averages;
    for (auto &i : averages) {
      auto &v = i.first;
      v[0].setZero();
      v[1].setConstant(10000);
      v[2].setZero();
      v[3].setZero();
      v[4].setConstant(10000);
      v[5].setZero();
    }
    /// write result stats
    std::fstream resultOut("/tmp/resultOut", std::ios::out);
    // status reprojErrMeanTest rmsTest
    for (const auto &result : resultList) {
      auto &first = averages[result.status].first;
      const auto rmsTestDbl = result.rmsTest.cast<double>();
      const auto rmsCalibDbl = result.rmsCalib.cast<double>();
      averages[result.status].first[0] += rmsTestDbl;
      averages[result.status].first[3] += rmsCalibDbl;

      first[1].x() = std::min(first[1].x(), rmsTestDbl.x());
      first[1].y() = std::min(first[1].y(), rmsTestDbl.y());
      first[2].x() = std::max(first[2].x(), rmsTestDbl.x());
      first[2].y() = std::max(first[2].y(), rmsTestDbl.y());

      first[4].x() = std::min(first[4].x(), rmsCalibDbl.x());
      first[4].y() = std::min(first[4].y(), rmsCalibDbl.y());
      first[5].x() = std::max(first[5].x(), rmsCalibDbl.x());
      first[5].y() = std::max(first[5].y(), rmsCalibDbl.y());

      averages[result.status].second++;

      resultOut << result.status << " " << result.reprojectionErrorMeanTest
                << " " << result.rmsTest.x() << " " << result.rmsTest.y() << " "
                << result.reprojectionErrorMeanCalib << " "
                << result.rmsCalib.x() << " " << result.rmsCalib.y() << "\n";
    }
    resultOut.flush();
    resultOut.close();

    for (size_t i = 0; i < averages.size(); i++) {
      const auto &elem = averages[i];
      if (elem.second) {
        std::cout << "RMS stats for status: "
                  << (i == jCalib::CalibStatus::SUCCESS
                          ? "Success"
                          : "Fail " + std::to_string(i))
                  << ": \n\tTest RMS ->  avg[ "
                  << (elem.first[0].transpose() / elem.second) << " ]\tmin[ "
                  << elem.first[1].transpose() << " ]\tmax[ "
                  << elem.first[2].transpose() << " ] "
                  << "\n\tCalib RMS -> avg[ "
                  << (elem.first[3].transpose() / elem.second) << " ]\tmin[ "
                  << elem.first[4].transpose() << " ]\tmax[ "
                  << elem.first[5].transpose() << " ]\n";
      }
    }
    std::cout << std::endl;
  }

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
