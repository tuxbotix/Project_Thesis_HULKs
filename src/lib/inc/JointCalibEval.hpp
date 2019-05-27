#pragma once

#include "JointCalibSolver.hpp"
#include <iomanip>

#ifndef DEBUG_SIM_TEST
#define DEBUG_SIM_TEST 0
#endif

namespace JointCalibEval {

using CombinedPoseListT =
    std::vector<std::pair<NaoPoseAndRawAngles<float>, Camera>>;

using CalibStatusStatistics =
    std::array<size_t, JointCalibration::CalibStatus::MAX_STATUS_COUNT>;

// template <typename T> struct CalibEvalResiduals {
//  JointCalibration::Residual<T> preX;
//  JointCalibration::Residual<T> preY;
//  JointCalibration::Residual<T> postX;
//  JointCalibration::Residual<T> postY;

//  std::array<JointCalibration::Residual<T>,
//             JointCalibSolvers::COMPACT_JOINT_CALIB_PARAM_COUNT>
//      jointResiduals;

//  /**
//*@brief joinEvalResiduals move-insert ops for all vectors. Src list is
//*joined to dest list
//*@param src source CalibEvalResidual
//*@param dest destination CalibEvalResidual
//*@return Reference to dest
//*/
//  CalibEvalResiduals &joinEvalResiduals(CalibEvalResiduals &src,
//                                        CalibEvalResiduals &dest) {
//    dest.preX.insert(dest.preX.end(),
//    std::make_move_iterator(src.preX.begin()),
//                     std::make_move_iterator(src.preX.end()));
//    dest.preY.insert(dest.preY.end(),
//    std::make_move_iterator(src.preY.begin()),
//                     std::make_move_iterator(src.preY.end()));
//    dest.postX.insert(dest.postX.end(),
//                      std::make_move_iterator(src.postX.begin()),
//                      std::make_move_iterator(src.postX.end()));
//    dest.postY.insert(dest.postY.end(),
//                      std::make_move_iterator(src.postY.begin()),
//                      std::make_move_iterator(src.postY.end()));
//    for (size_t i = 0; i < JointCalibSolvers::COMPACT_JOINT_CALIB_PARAM_COUNT;
//         ++i) {
//      auto &srcjoint = src.jointResiduals[i];
//      auto &dstjoint = dest.jointResiduals[i];
//      dstjoint.insert(dstjoint.end(),
//      std::make_move_iterator(srcjoint.begin()),
//                      std::make_move_iterator(srcjoint.end()));
//    }
//    return dest;
//  }
//};

/**
 * @brief The OutputData struct This is supposed to deprecate calibEvalResiduals
 */
template <typename T> struct OutputData {
  // Pre and Post should only be calib results..
  JointCalibration::Residual<T> preX;
  JointCalibration::Residual<T> preY;
  JointCalibration::Residual<T> postX;
  JointCalibration::Residual<T> postY;
  JointCalibration::Residual<T> jointResiduals;

  Eigen::VectorXf jointError;

  JointCalibration::JointCalibResult calibResult;
  CombinedPoseListT testPoses;
  CombinedPoseListT calibPoses;

  //  static void dumpToFile(const std::vector<OutputData> &data,
  //                         const std::string prefix) {
  //      std::cout<
  //  }
};

template <typename T>
/**
 * @brief The residualGroup struct This is only for statistics purposes.
 */
class ResidualGroup {
  using residualForAngles =
      std::array<T, JointCalibSolvers::COMPACT_JOINT_CALIB_PARAM_COUNT>;
  using residualListForAngles =
      std::array<JointCalibration::Residual<T>,
                 JointCalibSolvers::COMPACT_JOINT_CALIB_PARAM_COUNT>;

public:
  JointCalibration::ResidualStdVec<T> preX;
  JointCalibration::ResidualStdVec<T> preY;
  JointCalibration::ResidualStdVec<T> postX;
  JointCalibration::ResidualStdVec<T> postY;

  std::array<JointCalibration::ResidualStdVec<T>,
             JointCalibSolvers::COMPACT_JOINT_CALIB_PARAM_COUNT>
      jointResiduals;
  std::array<JointCalibration::ResidualStdVec<T>,
             JointCalibSolvers::COMPACT_JOINT_CALIB_PARAM_COUNT>
      inducedError;
  CalibStatusStatistics statistics;

  ResidualGroup()
      : preX(), preY(), postX(), postY(), jointResiduals(), statistics{0} {}

private:
public:
  void dumpToFiles(const std::string &prefix) {

    const auto prePostRes = "_prePostResiduals";
    //        {preY, "_originalResidualY"},
    //        {postX, "_calibratedResidualX"},
    //        {postY, "_calibratedResidualY"}};

    //    const auto calibJointRes = "_calibratedJointResidual";
    //    const auto uncalibJointRes = "_unCalibratedJointResidual";

    std::fstream out(prefix + prePostRes, std::ios::out);
    if (out) {
      for (const auto &data : {preX, preY, postX, postY}) {
        out.precision(4);
        std::stringstream ss;
        for (const auto &v : data) {
          ss << v << " ";
        }
        out << ss.str() << "\n";
        out.flush();
      }
      out.close();
    } else {
      std::cerr << "File " << (prefix + prePostRes) << "not open" << std::endl;
    }
    //    out.open(prefix + calibJointRes);
    //    for (const auto &data : {preX, preY, postX, postY}) {
    //      if (out) {
    //        out.precision(4);
    //        out << data.transpose() << "\n";
    //      }
    //      out.flush();
    //    }
  }

  /**
  *@brief joinEvalResiduals move-insert ops for all vectors. Src list is
  *joined to dest list
  *@param src source CalibEvalResidual
  *@param dest destination CalibEvalResidual
  *@return Reference to dest
  */
  void joinEvalResiduals(const std::vector<OutputData<T>> &src) {

    for (const auto &elem : src) {
      preX.reserve(preX.size() + elem.preX.size());
      copy(elem.preX.data(), elem.preX.data() + elem.preX.size(),
           back_inserter(preX));

      preY.reserve(preY.size() + elem.preY.size());
      copy(elem.preY.data(), elem.preY.data() + elem.preY.size(),
           back_inserter(preY));

      postX.reserve(postX.size() + elem.postX.size());
      copy(elem.postX.data(), elem.postX.data() + elem.postX.size(),
           back_inserter(postX));

      postY.reserve(postY.size() + elem.postY.size());
      copy(elem.postY.data(), elem.postY.data() + elem.postY.size(),
           back_inserter(postY));
      //      if (preX.size()) {
      //        JointCalibration::Residual<T> tempPreX;
      //        tempPreX << preX, elem.preX;
      //        std::swap(tempPreX, preX);
      //      } else {
      //        preX = elem.preX;
      //      }
      //      if (preY.size()) {
      //        JointCalibration::Residual<T> tempPreY;
      //        tempPreY << preY, elem.preY;
      //        std::swap(tempPreY, preY);
      //      } else {
      //        preY = elem.preY;
      //      }
      //      if (postX.size()) {
      //        JointCalibration::Residual<T> tempPoX;
      //        tempPoX << postX, elem.postX;
      //        std::swap(tempPoX, postX);
      //      } else {
      //        postX = elem.postX;
      //      }
      //      if (postY.size()) {
      //        JointCalibration::Residual<T> tempPoY;
      //        tempPoY << postY, elem.postY;
      //        std::swap(tempPoY, postY);
      //      } else {
      //        postY = elem.postY;
      //      }
      for (size_t i = 0; i < JointCalibSolvers::COMPACT_JOINT_CALIB_PARAM_COUNT;
           ++i) {
        //        if (jointResiduals[i].size()) {
        //          JointCalibration::Residual<T> temp;
        //          temp << jointResiduals[i], elem.jointResiduals;
        //          std::swap(temp, jointResiduals[i]);
        //        } else {
        //          jointResiduals[i] = elem.jointResiduals;
        //        }
        jointResiduals[i].emplace_back(elem.jointResiduals(i));
      }
      statistics[elem.calibResult.status]++;
      //      statistics[static_cast<size_t>(elem.calibResult.status)]++;
    }
  }

  std::tuple<residualForAngles, residualForAngles, residualForAngles>
  getMinMaxAvg() {
    std::array<T, JointCalibSolvers::COMPACT_JOINT_CALIB_PARAM_COUNT> minData =
        {0};
    std::array<T, JointCalibSolvers::COMPACT_JOINT_CALIB_PARAM_COUNT> maxData =
        {0};
    std::array<T, JointCalibSolvers::COMPACT_JOINT_CALIB_PARAM_COUNT> avgData =
        {0};
    for (size_t i = 0; i < JointCalibSolvers::COMPACT_JOINT_CALIB_PARAM_COUNT;
         ++i) {
      auto temp = jointResiduals[i]; //.template cast<double>();
      avgData[i] = static_cast<T>(
          temp.size() > 0
              ? std::accumulate(temp.begin(), temp.end(), T(0)) / T(temp.size())
              : T(0));
      auto minMaxPostX = std::minmax_element(temp.begin(), temp.end());
      minData[i] = static_cast<T>(temp.size() > 0 ? *minMaxPostX.first : T(0));
      maxData[i] = static_cast<T>(temp.size() > 0 ? *minMaxPostX.second : T(0));
    }
    return {minData, maxData, avgData};
  }

  void printStats(const T JOINT_CALIB_QUALITY_TOL_DEG,
                  const size_t JOINT_ERR_LST_COUNT) {
    auto data = getMinMaxAvg();
    auto &minData = std::get<0>(data);
    auto &maxData = std::get<1>(data);
    auto &avgData = std::get<2>(data);

    std::cout << std::endl;
    for (size_t i = 0; i < JointCalibSolvers::COMPACT_JOINT_CALIB_PARAM_COUNT;
         ++i) {
      //      tempHist.update(resVec);
      size_t badCount = 0;
      for (const auto &elem : jointResiduals[i]) {
        if (elem > JOINT_CALIB_QUALITY_TOL_DEG) {
          badCount++;
        }
      }
      //      size_t badCount =
      //          (jointResiduals[i].array() >
      //          JOINT_CALIB_QUALITY_TOL_DEG).count();
      std::cout << "Joint " << std::setw(18) << std::left
                << Sensor::JOINT_NAMES[Sensor::ALL_OBS_JOINTS[i]] << std::right
                << " \tbad: " << std::setw(6) << std::setprecision(2)
                << (badCount * 100.0 / static_cast<double>(JOINT_ERR_LST_COUNT))
                << "%\tmin: " << std::setw(6) << std::setprecision(2)
                << minData[i] << "\tmax: " << std::setw(6)
                << std::setprecision(2) << maxData[i]
                << "\tavg: " << std::setw(6) << std::setprecision(2)
                << avgData[i] << "\n";
    };
    std::cout
        << "\nTotal Bad cases: \t"
        << (std::accumulate(statistics.begin(), statistics.end(),
                            static_cast<size_t>(0)) -
            statistics[JointCalibration::CalibStatus::SUCCESS]) *
               100 / static_cast<T>(JOINT_ERR_LST_COUNT)
        << "%\n"
        << "FAIL_LOCAL_MINIMA:    \t"
        << statistics[JointCalibration::CalibStatus::FAIL_LOCAL_MINIMA] * 100 /
               static_cast<T>(JOINT_ERR_LST_COUNT)
        << "%\n"
        << "FAIL_NO_CONVERGE:     \t"
        << statistics[JointCalibration::CalibStatus::FAIL_NO_CONVERGE] * 100 /
               static_cast<T>(JOINT_ERR_LST_COUNT)
        << "%\n"
        << "FAIL_NUMERICAL:       \t"
        << statistics[JointCalibration::CalibStatus::FAIL_NUMERICAL] * 100 /
               static_cast<T>(JOINT_ERR_LST_COUNT)
        << "%\n"
        << "FAIL_NO_CAPTURES:     \t"
        << statistics[JointCalibration::CalibStatus::FAIL_NO_CAPTURES] * 100 /
               static_cast<T>(JOINT_ERR_LST_COUNT)
        << "%\n"
        << "FAIL_NO_TEST_CAPTURES:\t"
        << statistics[JointCalibration::CalibStatus::FAIL_NO_TEST_CAPTURES] *
               100 / static_cast<T>(JOINT_ERR_LST_COUNT)
        << "%\n"
        << "SUCCESS:              \t"
        << statistics[JointCalibration::CalibStatus::SUCCESS] * 100 /
               static_cast<T>(JOINT_ERR_LST_COUNT)
        << "%\n"
        << std::endl;
  }
};

struct JointErrorEval {

  const NaoJointAndSensorModelConfig cfg;
  const std::vector<CalibrationFeatures::CalibrationFeaturePtr<float>>
      calibrationFeaturePtrs;
  const CombinedPoseListT combinedPoseList;
  const CombinedPoseListT combinedTestPoseList;
  const SUPPORT_FOOT supFoot;
  const float jointCalibQualityTol;
  const float reprojErrTolPercent;
  const bool stochasticFix;
  const bool enablePixelNoise;
  const bool enableJointNoise;
  const float minJointErrVal;
  const float maxJointErrVal;

  /// Random generator
  std::default_random_engine generator; // random gen
  std::normal_distribution<float> pixelNoiseDistribution;
  std::normal_distribution<double> jointNoiseDistribution;

  JointErrorEval(
      const NaoJointAndSensorModelConfig cfg,
      const std::vector<CalibrationFeatures::CalibrationFeaturePtr<float>>
          &calibrationFeaturePtrs,
      const CombinedPoseListT poseList,
      const CombinedPoseListT combinedTestPoseList, const SUPPORT_FOOT supFoot,
      const float minJointErrVal, const float maxJointErrVal,
      const float jointCalibQualityTol, const float reprojErrTolPercent,
      const float pixelNoiseStdDev, const float jointNoiseStdDev,
      const bool stochasticFix, const bool enablePixelNoise,
      const bool enableJointNoise)
      : cfg(cfg), calibrationFeaturePtrs(calibrationFeaturePtrs),
        combinedPoseList(poseList), combinedTestPoseList(combinedTestPoseList),
        supFoot(supFoot), jointCalibQualityTol(jointCalibQualityTol),
        reprojErrTolPercent(reprojErrTolPercent), stochasticFix(stochasticFix),
        enablePixelNoise(enablePixelNoise), enableJointNoise(enableJointNoise),
        minJointErrVal(minJointErrVal), maxJointErrVal(maxJointErrVal),
        generator(), pixelNoiseDistribution(0.0f, pixelNoiseStdDev),
        jointNoiseDistribution(0.0, static_cast<double>(jointNoiseStdDev)) {}

  JointCalibSolvers::CaptureList
  getFrameCaptures(NaoJointAndSensorModel &naoJointSensorModel,
                   const rawPoseT inducedErrorStdVec, bool testFrames) {
    JointCalibSolvers::CaptureList frameCaptures;
    // Capture per pose.
    size_t iter = 0;
    auto &poseList = (testFrames ? combinedTestPoseList : combinedPoseList);
    for (const auto &combinedPoseAndAngles : poseList) {
      const auto &poseAndAngles = combinedPoseAndAngles.first;
      const auto &camName = combinedPoseAndAngles.second;
      naoJointSensorModel.setCalibValues(inducedErrorStdVec);

      // This is actually pose error (not reaching the given pose exactly)
      if (enableJointNoise) {
        rawAnglesT poseAngles = poseAndAngles.angles;
        for (auto &elem : poseAngles) {
          elem += static_cast<float>(jointNoiseDistribution(generator) *
                                     TO_RAD_DBL);
        }
        naoJointSensorModel.setPose(poseAngles, poseAndAngles.pose.supportFoot);
      } else {
        naoJointSensorModel.setPose(poseAndAngles.angles,
                                    poseAndAngles.pose.supportFoot);
      }
      //      for (const auto &camName : camNames)
      {
        bool proceed = false;
        // will be relative to support foot, easier to manage
        const auto groundGrid = naoJointSensorModel.getFilteredCalibFeatures(
            camName, calibrationFeaturePtrs, proceed);
        //            naoJointSensorModel.getGroundGrid(camName, proceed);
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
          //          std::lock_guard<std::mutex> lg(utils::mtx_cout_);
          logStream << "Obtaining ground grid failed, gridCount: "
                    << groundGrid.size() << " pose:" << iter << " "
                    << " cam:" << (camName == Camera::TOP ? "TOP" : "BOT")
                    << std::endl;
#endif
        }
      }
      iter++;
    }
    return frameCaptures;
  }
  /**
   * @brief evalJointErrorSet Evaluate calibration capability for an error
   * config.
   * THIS IS NOT THREAD SAFE
   * @param naoModel As mentioned in threadedFcn..
   * @param poseList
   * @param inducedErrorStdVec Joint error configuration
   * @return pre-post residuals residuals and more infos.
   */
  std::tuple<JointCalibration::JointCalibResult, Eigen::VectorXf,
             Eigen::VectorXf, Eigen::VectorXf, Eigen::VectorXf, Eigen::VectorXf>
  evalJointErrorSet(const rawPoseT inducedErrorStdVec) {

#if DEBUG_SIM_TEST
    std::stringstream logStream;
    logStream << "BEGIN\n";
#endif
    // Create result object
    JointCalibration::JointCalibResult result;
    auto &successStatus = result.status;

    // Map the error vector into an Eigen vector
    Eigen::VectorXf inducedErrorEigCompact;
    if (!JointCalibSolvers::rawPoseToJointCalibParams(inducedErrorStdVec,
                                                      inducedErrorEigCompact)) {
      std::cerr << "Convertion failed" << std::endl;
    }

    // Initialize residual of joint calibration (this isn't squared*)
    Eigen::VectorXf jointCalibResidual = inducedErrorEigCompact;

    // Make a copy of nao sensor model
    auto naoJointSensorModel = NaoJointAndSensorModel(cfg);
    // set the error state

    // List of captures
    JointCalibSolvers::CaptureList frameCaptures;
    frameCaptures =
        getFrameCaptures(naoJointSensorModel, inducedErrorStdVec, false);

    // List of captures
    JointCalibSolvers::CaptureList testFrameCaptures;
    testFrameCaptures =
        getFrameCaptures(naoJointSensorModel, inducedErrorStdVec, true);

    if (testFrameCaptures.size() <= 0) {
      std::lock_guard<std::mutex> lg(utils::mtx_cout_);
      std::cerr << "Empty test captures" << std::endl;
      successStatus = JointCalibration::CalibStatus::FAIL_NO_TEST_CAPTURES;
      return {result, {}, {}, {}, {}, {}};
    }
    // If no frames are captured, consider this as a failure
    if (frameCaptures.size() <= 0) {
      successStatus = JointCalibration::CalibStatus::FAIL_NO_CAPTURES;
#if DEBUG_SIM_TEST
      //    std::lock_guard<std::mutex> lg(utils::mtx_cout_);
      logStream << "No suitable frame captures !!\n";
      logStream << "END";
      std::lock_guard<std::mutex> lg(utils::mtx_cout_);
      std::cout << logStream.str() << std::endl;
#endif
      return {result, {}, {}, {}, {}, {}};
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

    auto testFunctor =
        JointCalibSolvers::JointCalibrator(testFrameCaptures, cfg);
    Eigen::NumericalDiff<JointCalibSolvers::JointCalibrator, Eigen::Central>
        testor(testFunctor);

    Eigen::VectorXf initialErrorVec(calibrator.values());
    Eigen::VectorXf initialTestErrorVec(testor.values());
    Eigen::VectorXf finalErrorVec(calibrator.values());
    Eigen::VectorXf finalTestErrorVec(testor.values());

    // Setting zero is very important.
    Eigen::VectorXf calibratedParams(static_cast<Eigen::Index>(
        JointCalibSolvers::COMPACT_JOINT_CALIB_PARAM_COUNT));
    calibratedParams.setZero();

    // Get state of errors before calibration
    calibrator(calibratedParams, initialErrorVec);
    testor(calibratedParams, initialTestErrorVec);

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
    testor(calibratedParams, finalTestErrorVec);

    // Just to note if the loop was done and broken with success
    //    bool loopedAndBroken = false;
    // Do stochastic fixing?
    if (stochasticFix
        //        && (calibratedParams.minCoeff() < minJointErrVal * TO_RAD_FLT
        //        ||
        //         calibratedParams.maxCoeff() > maxJointErrVal * TO_RAD_FLT)
        ) {
      // This will hold best params in case looping is needed
      Eigen::VectorXf oldParams(static_cast<Eigen::Index>(
          JointCalibSolvers::COMPACT_JOINT_CALIB_PARAM_COUNT));
      Eigen::VectorXf oldErrorVec(finalErrorVec);

      size_t count = 0; // counter for the while loop

      std::uniform_real_distribution<float> distribution(minJointErrVal,
                                                         maxJointErrVal);

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
      while (count < 100) {
        // save state
        oldParams = calibratedParams;
        oldErrorVec = finalErrorVec;
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

        /// go back to previous parameter set.
        if (finalCost < finalErrorVec.squaredNorm()) {
          calibratedParams = oldParams;
          finalErrorVec = oldErrorVec;
        }
        count++;
      }

      testor(calibratedParams, finalTestErrorVec);

#if DEBUG_SIM_TEST
      {
        //      std::lock_guard<std::mutex> lg(utils::mtx_cout_);
        logStream << "i " << count << std::endl;
      }
#endif
    }

    jointCalibResidual -= calibratedParams; // Joint residual

    /// Initialize reprojection errors.
    //    Eigen::VectorXf reprojErrCalibInitial(initialErrorVec.size() / 2);
    Eigen::VectorXf reprojErrCalib(finalErrorVec.size() / 2);
    //    Eigen::VectorXf reprojErrTestInitial(initialErrorVec.size() / 2);
    Eigen::VectorXf reprojErrTest(finalTestErrorVec.size() / 2);
    //    Eigen::VectorXf reprojectionError;
    {

      if (initialTestErrorVec.size() != finalTestErrorVec.size()) {
        std::cerr << "Some major issue, error vector length mismatch"
                  << std::endl;
      }
      for (auto i = 0; i < finalTestErrorVec.size() / 2; ++i) {
        Eigen::Ref<Eigen::Vector2f> block = finalTestErrorVec.segment(i * 2, 2);
        reprojErrTest(i) = block.norm();
        //        block = initialTestErrorVec.segment(i * 2, 2);
        //        reprojErrTestInitial(i) = block.norm();
      }
      for (auto i = 0; i < finalErrorVec.size() / 2; ++i) {
        Eigen::Ref<Eigen::Vector2f> blockCalib =
            finalErrorVec.segment(i * 2, 2);
        reprojErrCalib(i) = blockCalib.norm();
        //        block = initialErrorVec.segment(i * 2, 2);
        //        reprojErrCalibInitial(i) = block.norm();
      }
    }

    // Get min-max info for error checking
    //    const auto reprojErrMin = reprojErrTest.minCoeff();
    //    const auto reprojErrMax = reprojErrTest.maxCoeff();

    const auto reprojErrorMean = reprojErrTest.mean();
    const auto reprojErrorNorm = reprojErrTest.norm();

    const auto jointCalibResAbsMax =
        std::max(std::abs(jointCalibResidual.maxCoeff()),
                 std::abs(jointCalibResidual.minCoeff()));
    const auto imageSize = naoJointSensorModel.getImSize();

    bool convergeOk =
        reprojErrorMean <= imageSize.minCoeff() * reprojErrTolPercent &&
        finalTestErrorVec.norm() <= initialTestErrorVec.norm();
    // Determine success or failure
    if (finalErrorVec.hasNaN() || finalTestErrorVec.hasNaN()) {
      successStatus = JointCalibration::CalibStatus::FAIL_NUMERICAL;
    } else if (convergeOk && jointCalibResAbsMax > jointCalibQualityTol) {
      successStatus = JointCalibration::CalibStatus::FAIL_LOCAL_MINIMA;
    } else if (!convergeOk) {
      successStatus = JointCalibration::CalibStatus::FAIL_NO_CONVERGE;
    } else {
      successStatus = JointCalibration::CalibStatus::SUCCESS;
    }

// if failed OR if loop broken BUT calibration is bad
/*
 * NOTE: Local minima if reprojection errr is small but jointResidual isn't
 * low!
 */
#if DEBUG_SIM_TEST
    if (successStatus != CalibStatus::SUCCESS ||
        (stochasticFix && jointCalibResAbsMax > jointCalibQualityTol)) {

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
    result.status = successStatus;
    result.jointParams.resize(JOINTS::JOINT::JOINTS_MAX);
    if (!JointCalibSolvers::jointCalibParamsToRawPose(calibratedParams,
                                                      result.jointParams)) {
    }
    result.reprojectionErrorNormCalib = reprojErrCalib.norm();
    result.reprojectionErrorMeanCalib = reprojErrCalib.mean();
    result.reprojectionErrorNormTest = reprojErrorNorm;
    result.reprojectionErrorMeanTest = reprojErrorMean;
    // calculate RMS
    {
      Eigen::Array2d rms = {0, 0};
      for (auto i = 0; i < finalTestErrorVec.size(); i += 2) {
        Eigen::Ref<Eigen::Array2f> block = finalTestErrorVec.segment(i, 2);

        rms += block.cast<double>().square();
      }
      result.rmsTest =
          (rms / (finalTestErrorVec.size() / 2)).sqrt().cast<float>();
      rms.setZero();
      for (auto j = 0; j < finalErrorVec.size(); j += 2) {
        Eigen::Ref<Eigen::Array2f> block = finalErrorVec.segment(j, 2);
        rms += block.cast<double>().square();
      }
      result.rmsCalib = (rms / (finalErrorVec.size() / 2)).sqrt().cast<float>();
    }
    // TODO std dev
    result.sampleSizeTest = static_cast<size_t>(reprojErrTest.size());
    result.sampleSizeCalib = static_cast<size_t>(reprojErrCalib.size());
#if DEBUG_SIM_TEST
    logStream << "END";
    std::lock_guard<std::mutex> lg(utils::mtx_cout_);
    std::cout << logStream.str() << std::endl;
#endif
    // Result, jointCalibResidual, reprojection error initial (test data),
    // reprojection error after calib.(test data), reprojection error initial
    // (calib dataset) reprojection error after calib (calib)
    return {result,
            jointCalibResidual,
            initialTestErrorVec,
            finalTestErrorVec,
            initialErrorVec,
            finalErrorVec};
  }
};

} // namespace JointCalibEval
