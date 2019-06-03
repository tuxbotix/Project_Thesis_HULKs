#define DEBUG_JOINT_AND_SENS 0
#define DEBUG_SIM_TEST 0
#define DO_COMMIT 1
#define WRITE_PARALLEL 1
#define DEBUG_FILE_COMMIT 1
#define DEBUG_IN_THREAD 1
#define ENABLE_PROGRESS 1

#include "CalibrationFeatures.hpp"
#include "JointCalibEval.hpp"
#include "JointCalibSolver.hpp"
#include "MiniConfigHandle.hpp"
#include "NaoPoseInfo.hpp"
#include "TUHHMin.hpp"
#include "constants.hpp"
#include "utils.hpp"

#include <Data/CameraMatrix.hpp>
#include <cxxopts/include/cxxopts.hpp>

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

const unsigned int MAX_THREADS = std::thread::hardware_concurrency();

namespace jCalib = JointCalibration;
namespace jCalibSolver = JointCalibSolvers;
namespace jCEval = JointCalibEval;

using CalibResultWithResidualVec = std::vector<jCEval::OutputData<float>>;
using StatisticsWithResults =
    std::pair<jCEval::CalibStatusStatistics, CalibResultWithResidualVec>;

/**
 * @brief threadedFcn
 * @param combinedPoseList
 * @param combinedTestPoseList
 * @param supFoot
 * @param cfg
 * @param calibrationFeaturePtrs
 * @param jointErrItrBegin
 * @param jointErrItrEnd
 * @param minJointErrVal
 * @param maxJointErrVal
 * @param jointCalibQualityTol
 * @param reprojErrTolPercent
 * @param pixelNoiseStdDev
 * @param jointNoiseStdDev
 * @param jointSampleCount
 * @param stochasticFix
 * @param enablePixelNoise
 * @param enableJointNoise
 * @return
 */
StatisticsWithResults
threadedFcn(const jCEval::CombinedPoseListT &combinedPoseList,
            const jCEval::CombinedPoseListT &combinedTestPoseList,
            const SUPPORT_FOOT &supFoot,
            const NaoJointAndSensorModelConfig &cfg,
            const std::vector<CalibrationFeatures::CalibrationFeaturePtr<float>>
                &calibrationFeaturePtrs,
            const std::vector<rawPoseT>::iterator &jointErrItrBegin,
            const std::vector<rawPoseT>::iterator &jointErrItrEnd,
            const float &minJointErrVal, const float &maxJointErrVal,
            const float &jointCalibQualityTol, const float &reprojErrTolPercent,
            const float &pixelNoiseStdDev, const float &jointNoiseStdDev,
            const size_t &jointSampleCount, const bool &stochasticFix,
            const bool &enablePixelNoise, const bool &enableJointNoise) {

  jCEval::CalibStatusStatistics calibStatusStatistics = {};
  CalibResultWithResidualVec resultList;
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
      cfg, calibrationFeaturePtrs, combinedPoseList, combinedTestPoseList,
      supFoot, minJointErrVal, maxJointErrVal, jointCalibQualityTol,
      reprojErrTolPercent, pixelNoiseStdDev, jointNoiseStdDev, jointSampleCount,
      stochasticFix, enablePixelNoise, enableJointNoise);

  for (auto errSetIter = jointErrItrBegin; errSetIter != jointErrItrEnd;
       ++errSetIter) {

    auto &errorSet = *errSetIter;

    JointCalibEval::OutputData<float> output;

    auto &result = output.calibResult;
    auto &jointCalibResiduals = output.jointResiduals;

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

    auto prePostResSize = static_cast<Eigen::Index>(preTestResiduals.size());
    //    auto curPreResSize = residuals.preX.size();

    // reserve vector memory to speed up the process
    //    output.preX.resize(prePostResSize);
    //    output.postX.resize(prePostResSize);
    //    output.preY.resize(prePostResSize);
    //    output.postY.resize(prePostResSize);

    //    for (Eigen::Index i = 0, odd = 0, even = 0; i <
    //    preTestResiduals.size();
    //         ++i) {
    //      if (i % 2 == 0) { // x -> even number
    //        output.preX(even) = preTestResiduals(i);
    //        output.postX(even) = postTestResiduals(i);
    //        even++;
    //      } else { // odd  -> newline
    //        output.preY(odd) = preTestResiduals(i);
    //        output.postY(odd) = postTestResiduals(i);
    //        odd++;
    //      }
    //    }
    // Convert to degrees
    jointCalibResiduals /= TO_RAD_FLT;
    output.jointResiduals = jointCalibResiduals;
    {
      if (!JointCalibSolvers::rawPoseToJointCalibParams(errorSet,
                                                        output.jointError)) {
      }
      output.jointError /= TO_RAD_FLT;
    }
    //    for (Eigen::Index i = 0;
    //         i < JointCalibSolvers::COMPACT_JOINT_CALIB_PARAM_COUNT; ++i) {
    //      // TODO FIX THIS
    //      //      residuals.jointResiduals[static_cast<size_t>(i)].push_back(
    //      //          jointCalibResiduals(i));
    //      residuals.jointResiduals[static_cast<size_t>(i)].push_back(
    //          static_cast<double>(jointCalibResiduals(i)));
    //    }
    calibStatusStatistics[result.status]++;
    resultList.emplace_back(std::move(output));
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
  options.add_options()("f,fileInput", "Input File prefix",
                        cxxopts::value<std::string>())(
      "o,fileOutput", "Output File prefix",
      cxxopts::value<std::string>()->default_value(utils::getISOTimeString() +
                                                   "_simOut"))(
      "t,testFile", "testPoseFile", cxxopts::value<std::string>())(
      "e,errorConfigs", "set of error cases",
      cxxopts::value<std::string>()->default_value("./l_1000_jointErrors.txt"))(
      "n,n-poses", "Poses to calibrate",
      cxxopts::value<size_t>()->default_value("10"))(
      "testErrorCount", "Error configs to try. Defaults to values in json..",
      cxxopts::value<int64_t>()->default_value("-1"))(
      "m,mirror", "Mirror the poses",
      cxxopts::value<bool>()->default_value("false"))(
      "mirrorDouble", "Test both legs with poses for one",
      cxxopts::value<bool>()->default_value("false"))(
      "confRoot", "conf path",
      cxxopts::value<std::string>()->default_value("./"))(
      "calib", "calib features file",
      cxxopts::value<std::string>()->default_value("calibFeatures"))(
      "naoConfRoot", "Nao Conf path",
      cxxopts::value<std::string>()->default_value("../../nao/home/"))(
      "s,stochastic", "Stochastic mode?",
      cxxopts::value<bool>()->default_value("false"))(
      "h,histogram", "show histogram",
      cxxopts::value<bool>()->default_value("false"))(
      "j,jointNoise", "apply joint noise",
      cxxopts::value<bool>()->default_value("false"))(
      "p,pixelNoise", "apply pixel noise",
      cxxopts::value<bool>()->default_value("false"))(
      "jointSampleCount", "Joint sample count",
      cxxopts::value<size_t>()->default_value("1"));

  auto result = options.parse(argc, argv);
  const int64_t testErrorCount = result["testErrorCount"].as<int64_t>();
  const std::string inFileName = result["fileInput"].as<std::string>();
  const std::string outFilePrefix = result["fileOutput"].as<std::string>();
  const std::string inTestFileName = result["testFile"].as<std::string>();
  const std::string calibFeatureFileName = result["calib"].as<std::string>();
  const std::string errConfigFileName =
      result["errorConfigs"].as<std::string>();
  const size_t MAX_POSES_TO_CALIB = result["n-poses"].as<size_t>();
  // Default is false
  const bool mirrorPose = result["mirror"].as<bool>();
  const bool mirrorDouble =
      result["mirrorDouble"]
          .as<bool>(); // mirror given poses to test for both legs
  const std::string naoConfRoot = result["naoConfRoot"].as<std::string>();
  const std::string confRoot = result["confRoot"].as<std::string>();
  // Default is false
  const bool stochasticFix = result["stochastic"].as<bool>();
  const bool showHistogram = result["histogram"].as<bool>();
  const bool enablePixelNoise = result["pixelNoise"].as<bool>();
  const bool enableJointNoise = result["jointNoise"].as<bool>();
  const size_t jointSampleCount =
      result["jointSampleCount"].as<size_t>(); // how many times to sample joint
                                               // angle readings and average it

  std::fstream inFile(inFileName);
  if (!inFile.is_open()) {
    std::cerr << "Input file " << inFileName << "cannot be opened, exiting.."
              << std::endl;
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
  if (!MiniConfigHandle::mountFile(
          confRoot + "configuration/simGridEvalConf.json", confValue)) {
    std::cout << "couldn't open the conf file" << std::endl;
    exit(1);
  }
  /// observerModel config (to get grpound grid info, etc)
  Uni::Value obsModelConfig;
  if (!MiniConfigHandle::mountFile(
          confRoot + "configuration/cameraObsModelConf.json", obsModelConfig)) {
    std::cout << "couldn't open the conf file" << std::endl;
    exit(1);
  }

  const float MIN_ERR_VAL =
      static_cast<float>(confValue["errRangeMinMax"].at(0).asDouble());
  const float MAX_ERR_VAL =
      static_cast<float>(confValue["errRangeMinMax"].at(1).asDouble());
  /// Sample Size. Defaults to config. else overridden by errorCount param
  const size_t JOINT_ERR_LST_DESIRED_COUNT = static_cast<size_t>(
      testErrorCount > 0 ? testErrorCount
                         : confValue["testErrorCount"].asInt64());
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
  TUHH tuhhInstance(naoConfRoot);
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
  {
    const size_t actualPoseCount =
        mirrorDouble ? MAX_POSES_TO_CALIB * 2 : MAX_POSES_TO_CALIB;
    poseList.reserve(actualPoseCount);
    poseAndRawAngleT poseAndAngles;

    for (; poseList.size() <= actualPoseCount &&
           JointsAndPosesStream::getNextPoseAndRawAngles(inFile,
                                                         poseAndAngles);) {
      if (poseAndAngles.pose.supportFoot == SUPPORT_FOOT::SF_NONE) {
        continue;
      }
      if (!mirrorPose) {
        // as it is
        poseList.push_back(poseAndAngles);
      }
      // Mirror this pose. Nice in testing ;)
      if (mirrorDouble || mirrorPose) {
        poseAndAngles.mirror();
        poseList.push_back(poseAndAngles);
      }

      // if already double, skip
      if (supFeet != SUPPORT_FOOT::SF_DOUBLE) {
        // if none, first try..
        if (supFeet == SUPPORT_FOOT::SF_NONE) {
          supFeet = poseAndAngles.pose.supportFoot;
        } else if (mirrorDouble ||
                   supFeet !=
                       poseAndAngles.pose.supportFoot) { // if differs but
                                                         // not none, then
                                                         // double it is ;)
          supFeet = SUPPORT_FOOT::SF_DOUBLE;
        }
      }
    }
  }
  if (supFeet == SUPPORT_FOOT::SF_NONE) {
    std::cerr << "No valid support feet found!! exiting..." << std::endl;
  }
  std::cout << " reading of poses done, SF: " << supFeet << std::endl;

  /// Add test poses
  poseAndRawAngleListT testPoseList;
  {
    const size_t actualTestPoseCount =
        mirrorDouble ? std::max(size_t(100), testPoseCount) * 2
                     : std::max(size_t(100), testPoseCount);
    poseAndRawAngleT testPoseAndAngles;
    for (size_t i = 0; i < actualTestPoseCount &&
                       testPoseList.size() <= actualTestPoseCount &&
                       JointsAndPosesStream::getNextPoseAndRawAngles(
                           inTestFile, testPoseAndAngles);
         ++i) {
      /// TODO TEMP add support for double foot
      if (testPoseAndAngles.pose.supportFoot == SUPPORT_FOOT::SF_NONE) {
        continue;
      }
      // as it is
      if (!mirrorPose && (SUPPORT_FOOT::SF_DOUBLE == supFeet ||
                          testPoseAndAngles.pose.supportFoot == supFeet)) {
        testPoseList.push_back(testPoseAndAngles);
      }
      //    // Mirror this pose. Nice in testing ;)
      if (mirrorDouble || mirrorPose) {
        testPoseAndAngles.mirror();
        if (SUPPORT_FOOT::SF_DOUBLE == supFeet ||
            testPoseAndAngles.pose.supportFoot == supFeet) {
          testPoseList.push_back(testPoseAndAngles);
        } else {
          std::cerr << "Skipping" << std::endl;
        }
      }
    }
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

  // This will contain all above at the end. (move-join)
  jCEval::ResidualGroup<float> finalResidualSet;

  // vector holding the std::threads
  std::vector<std::future<StatisticsWithResults>>

      //  std::vector<std::future<std::pair<jCalib::CalibStatusStatistics,
      //                                    std::vector<jCalib::JointCalibResult>>>>
      futureList(MAX_THREADS);

  // Setup offsets for iterators
  size_t threadingOffsets = JOINT_ERR_LST_COUNT / MAX_THREADS;

  // cameras to evaluate
  std::vector<Camera> camNames = {Camera::BOTTOM, Camera::TOP};

  std::vector<CalibrationFeatures::CalibrationFeaturePtr<float>>
      calibrationFeatures;

  {
    std::fstream calibFeatureFile(calibFeatureFileName, std::ios::in);
    if (!calibFeatureFile) {
      std::cerr << "Calib feature file " << calibFeatureFileName
                << " not available. Exciting.." << std::endl;
      exit(1);
    }
    std::string line;
    while (calibFeatureFile.good() && std::getline(calibFeatureFile, line)) {
      std::stringstream ss(line);
      try {

        calibrationFeatures.emplace_back(
            CalibrationFeatures::deserializeCalibFeature(ss));
      } catch (...) {
        std::cerr << "Deserialize failed: " << line << std::endl;
      }
    }

    std::fstream file(outFilePrefix + "_calibPoints.txt", std::ios::out);
    file << "point_x point_y" << std::endl;
    for (const auto &feat : calibrationFeatures) {
      auto pts = feat->getGroundPoints();
      for (const auto &pt : pts) {
        file << pt.x() << " " << pt.y() << "\n";
      }
    }
    file.flush();
    file.close();
    std::cout << "Save chessboard points" << std::endl;
  }

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
    //                jointSampleCount
    //                bool stochasticFix, const bool enablePixelNoise,
    //                const bool enableJointNoise)
    futureList[i] = std::async(
        std::launch::async, &threadedFcn, combinedPoseList,
        combinedTestPoseList, supFeet, cfg, std::ref(calibrationFeatures),
        begin, end, MIN_ERR_VAL, MAX_ERR_VAL, JOINT_CALIB_QUALITY_TOL_RAD,
        REPROJ_ERR_TOL_PERCENT, PIXEL_NOISE_STD_DEV, JOINT_NOISE_STD_DEV,
        jointSampleCount, stochasticFix, enablePixelNoise, enableJointNoise);
  }

  /*
   * Finish calibration - join threads
   * Also join all residuals into finalResidualSet
   * + calibration results!!
   */
  jCEval::CalibStatusStatistics stats = {};
  CalibResultWithResidualVec resultList;
  for (size_t i = 0; i < MAX_THREADS; ++i) {
    auto &future = futureList[i];
    //    auto &residual = calibResidualList[i];
    std::cout << "get future" << i << std::endl;
    auto temp = future.get();
    std::cout << "got " << utils::getMilliSecondsString() << std::endl;
    for (size_t j = 0; j < stats.size(); ++j) {
      stats[j] += temp.first[j];
    }
    // append do the result vector
    resultList.insert(resultList.end(), temp.second.begin(), temp.second.end());
    finalResidualSet.joinEvalResiduals(temp.second);
    std::cout << "joined Data" << i << std::endl;
  }
  std::cout << "Processing done" << std::endl;

  /*
   * Print stats for each joint
   */
  finalResidualSet.printStats(JOINT_CALIB_QUALITY_TOL_DEG, JOINT_ERR_LST_COUNT);

  /*
   * Post-processing -> Get min-max bounds
   */
  //  std::cout << "Initializing histograms\n";
  //  auto minMaxPostX = std::minmax_element(finalResidualSet.postX.begin(),
  //                                         finalResidualSet.postX.end());

  //  auto minMaxPostY = std::minmax_element(finalResidualSet.postY.begin(),
  //                                         finalResidualSet.postY.end());

  //  auto maxOfJointParamsAll = [&finalResidualSet] {
  //    auto max = 0.0;
  //    for (const auto &vec : finalResidualSet.jointResiduals) {
  //      auto v = std::minmax_element(vec.begin(), vec.end());
  //      max = std::max(max, std::max(std::abs(*v.first),
  //      std::abs(*v.second)));
  //    }
  //    return max;
  //  }();

  //  //  auto maxOfJointParamsAll =
  //  //      1.01 * std::max(std::abs(*minMaxJointParams.first),
  //  //                      std::abs(*minMaxJointParams.second));
  //  auto maxOfAll = std::max(std::abs(*minMaxPostX.first),
  //  *minMaxPostX.second);
  //  maxOfAll = 1.01 * std::max(std::max(std::abs(*minMaxPostY.first),
  //                                      *minMaxPostY.second),
  //                             maxOfAll);

  //  std::cout << "AbsMax Post Errors: " << maxOfAll
  //            << "\nAbsMax Joint Errors: " << maxOfJointParamsAll <<
  //            std::endl;

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

    // Write result stats
    std::fstream resultOut(outFilePrefix + "_resultOut", std::ios::out);
    resultOut << "status reprojectionErrorMeanTest rmsTest_x rmsTest_y "
                 "reprojectionErrorMeanCalib rmsCalib_x rmsCalib_y ";
    for (size_t i = 0; i < finalResidualSet.jointResiduals.size(); ++i) {
      resultOut << "res_" << Sensor::JOINT_NAMES[Sensor::ALL_OBS_JOINTS[i]]
                << " ";
    }
    for (size_t i = 0; i < finalResidualSet.jointResiduals.size(); ++i) {
      resultOut << "err_" << Sensor::JOINT_NAMES[Sensor::ALL_OBS_JOINTS[i]]
                << " ";
    }
    resultOut << std::endl;

    // status reprojErrMeanTest rmsTest
    for (const auto &outputData : resultList) {
      const auto &result = outputData.calibResult;
      const auto &residual = outputData.jointResiduals;
      const auto &origError = outputData.jointError;

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
                << result.rmsCalib.x() << " " << result.rmsCalib.y() << " "
                << residual.transpose() << " " << origError.transpose() << "\n";
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

    //    finalResidualSet.dumpToFiles(outFilePrefix);
  }

  //  if (!showHistogram) {
  //    return 0;
  //  }

  //  /*
  //   * Initialize Histograms
  //   */
  //  utils::SimpleHistogram<double> preXhist(1000, -300, 300);
  //  utils::SimpleHistogram<double> preYhist(1000, -300, 300);

  //  utils::SimpleHistogram<double> postXhist(100, -maxOfAll, maxOfAll);
  //  utils::SimpleHistogram<double> postYhist(100, -maxOfAll, maxOfAll);

  //  utils::SimpleHistogram<double> preParamhist(
  //      30, static_cast<double>(-MAX_ERR_VAL),
  //      static_cast<double>(MAX_ERR_VAL));
  //  utils::SimpleHistogram<double> postParamhist(300, -maxOfJointParamsAll,
  //                                               maxOfJointParamsAll);

  //  for (const auto &elem : jointErrList) {
  //    std::vector<double> newElem;
  //    for (auto &i : elem) {
  //      float val = i / TO_RAD_FLT;
  //      if (val > 0.01f || val < -0.01f) {
  //        newElem.push_back(static_cast<double>(val));
  //      }
  //    }
  //    preParamhist.update(newElem);
  //  }
  //  // Update each histogram
  //  for (auto &elem : calibResidualList) {
  //    preXhist.update(elem.preX);
  //    preYhist.update(elem.preY);
  //    postXhist.update(elem.postX);
  //    postYhist.update(elem.postY);

  //    // For the moment, all joints are bundled together
  //    // TODO seperate this
  //    for (auto &jointErr : elem.jointResiduals) {
  //      std::vector<double> newElem;
  //      for (auto &i : jointErr) {
  //        if (i > __DBL_EPSILON__ || i < -__DBL_EPSILON__) {
  //          newElem.push_back(i);
  //        }
  //      }
  //      postParamhist.update(newElem);
  //    }
  //  }

  //  std::pair<utils::SimpleHistogram<double> &, std::string>
  //      originalResidualDumpX(preXhist, outFilePrefix + "_originalResidualX");
  //  std::pair<utils::SimpleHistogram<double> &, std::string>
  //      calibratedResidualDumpX(postXhist,
  //                              outFilePrefix + "_calibratedResidualX");
  //  std::pair<utils::SimpleHistogram<double> &, std::string>
  //      originalResidualDumpY(preYhist, outFilePrefix + "_originalResidualY");
  //  std::pair<utils::SimpleHistogram<double> &, std::string>
  //      calibratedResidualDumpY(postYhist,
  //                              outFilePrefix + "_calibratedResidualY");

  //  std::pair<utils::SimpleHistogram<double> &, std::string>
  //      calibratedJointParamResidualDumpY(
  //          postParamhist, outFilePrefix + "_calibratedJointResidual");

  //  std::pair<utils::SimpleHistogram<double> &, std::string>
  //  unCalibJointParams(
  //      preParamhist, outFilePrefix + "_unCalibratedJointResidual");

  //  for (auto elem : {originalResidualDumpX, originalResidualDumpY,
  //                    calibratedResidualDumpX, calibratedResidualDumpY}) {
  //    std::cout << "\nStart dump to : " << elem.second << "\n";
  //    std::pair<double, double> bounds = elem.first.getPercentileBounds(0.25);
  //    std::cout << "75% Of Population in: " << bounds.first << " "
  //              << bounds.second << "\n";
  //    bounds = elem.first.getPercentileBounds(0.1);
  //    std::cout << "90% Of Population in: " << bounds.first << " "
  //              << bounds.second << "\n";
  //    bounds = elem.first.getPercentileBounds(0.05);
  //    std::cout << "95% Of Population in: " << bounds.first << " "
  //              << bounds.second << "\n";
  //    bounds = elem.first.getPercentileBounds(0.01);
  //    std::cout << "99% Of Population in: " << bounds.first << " "
  //              << bounds.second << "\n";
  //    bounds = elem.first.getPercentileBounds(0.001);
  //    std::cout << "99.9% Of Population in: : " << bounds.first << " "
  //              << bounds.second << std::endl;

  //    // Open files, stream and close
  //    std::fstream file(elem.second, std::ios::out);
  //    file << elem.first;
  //    file.close();
  //  }

  //  /*
  //   * Put joint residuals seperately for real box plots and other stuff :P
  //   */
  //  for (auto elem : {unCalibJointParams, calibratedJointParamResidualDumpY})
  //  {
  //    std::cout << "\nStart dump to : " << elem.second << "\n";
  //    std::pair<double, double> bounds = elem.first.getPercentileBounds(0.25);
  //    std::cout << "75% Of Population in: " << bounds.first << " "
  //              << bounds.second << "\n";
  //    bounds = elem.first.getPercentileBounds(0.1);
  //    std::cout << "90% Of Population in: " << bounds.first << " "
  //              << bounds.second << "\n";
  //    bounds = elem.first.getPercentileBounds(0.05);
  //    std::cout << "95% Of Population in: " << bounds.first << " "
  //              << bounds.second << "\n";
  //    bounds = elem.first.getPercentileBounds(0.025);
  //    std::cout << "97.5% Of Population in: " << bounds.first << " "
  //              << bounds.second << "\n";
  //    bounds = elem.first.getPercentileBounds(0.01);
  //    std::cout << "99% Of Population in: : " << bounds.first << " "
  //              << bounds.second << std::endl;

  //    // Open files, stream and close
  //    std::fstream file(elem.second, std::ios::out);
  //    {
  //      for (const auto &residualSet : finalResidualSet.jointResiduals) {
  //        for (auto p = residualSet.begin(); p != residualSet.end() - 1; ++p)
  //        {
  //          file << *p << ", ";
  //        }
  //        if (residualSet.size() > 0) {
  //          file << *residualSet.end() << std::endl;
  //        } else {
  //          file << std::endl;
  //        }
  //      }
  //    }
  //    file.close();
  //  }

  return 0;
}
