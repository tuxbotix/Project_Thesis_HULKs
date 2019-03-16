#pragma once

#include "JointCalibSolver.hpp"

#ifndef DEBUG_SIM_TEST
#define DEBUG_SIM_TEST 0
#endif

namespace JointCalibEval {

using CombinedPoseListT =
    std::vector<std::pair<NaoPoseAndRawAngles<float>, Camera>>;

struct JointErrorEval {

  const NaoJointAndSensorModelConfig cfg;
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

  JointErrorEval(const NaoJointAndSensorModelConfig cfg,
                 const CombinedPoseListT poseList,
                 const CombinedPoseListT combinedTestPoseList,
                 const SUPPORT_FOOT supFoot, const float minJointErrVal,
                 const float maxJointErrVal, const float jointCalibQualityTol,
                 const float reprojErrTolPercent, const float pixelNoiseStdDev,
                 const float jointNoiseStdDev, const bool stochasticFix,
                 const bool enablePixelNoise, const bool enableJointNoise)
      : cfg(cfg), combinedPoseList(poseList),
        combinedTestPoseList(combinedTestPoseList), supFoot(supFoot),
        jointCalibQualityTol(jointCalibQualityTol),
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
      //      for (const auto &camName : camNames)
      {
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
             Eigen::VectorXf, Eigen::VectorXf, Eigen::VectorXf>
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
      std::cout << "empty test captures" << std::endl;
      successStatus = JointCalibration::CalibStatus::FAIL_NO_TEST_CAPTURES;
      return {result, {}, {}, {}, {}};
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
      return {result, {}, {}, {}, {}};
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

    auto testFunctor = JointCalibSolvers::JointCalibrator(frameCaptures, cfg);
    Eigen::NumericalDiff<JointCalibSolvers::JointCalibrator, Eigen::Central>
        testor(testFunctor);

    Eigen::VectorXf initialError(calibrator.values());
    Eigen::VectorXf finalErrorVec(calibrator.values());
    Eigen::VectorXf finalTestErrorVec(calibrator.values());

    // Setting zero is very important.
    Eigen::VectorXf calibratedParams(static_cast<Eigen::Index>(
        JointCalibSolvers::COMPACT_JOINT_CALIB_PARAM_COUNT));
    calibratedParams.setZero();

    // Get state of errors before calibration
    calibrator(calibratedParams, initialError);

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
      while (count < 20) {
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

    // Get min-max info for error checking
    const auto testMinCoeff = finalTestErrorVec.minCoeff();
    const auto testMaxCoeff = finalTestErrorVec.maxCoeff();

    const auto finalTestErrorAvg = finalTestErrorVec.mean();
    const auto finalTestErrorNorm = finalTestErrorVec.norm();

    // Get min-max info for error checking
    const auto minCoeff = finalErrorVec.minCoeff();
    const auto maxCoeff = finalErrorVec.maxCoeff();
    //  const auto finalErrAbsMax = std::max(std::abs(minCoeff),
    //  std::abs(maxCoeff));
    const auto finalErrorAvg = finalErrorVec.mean();
    const auto finalErrorNorm = finalErrorVec.norm();
    //  const auto errorAvg = finalErrorVec.norm() / finalErrorVec.size();
    const auto jointCalibResAbsMax =
        std::max(std::abs(jointCalibResidual.maxCoeff()),
                 std::abs(jointCalibResidual.minCoeff()));
    const auto imageSize = naoJointSensorModel.getImSize();

    // Determine success or failure
    if (finalTestErrorAvg <= imageSize.minCoeff() * reprojErrTolPercent &&
        finalErrorNorm <= initialError.norm() &&
        jointCalibResAbsMax > jointCalibQualityTol) {
      successStatus = JointCalibration::CalibStatus::FAIL_LOCAL_MINIMA;
    } else if (std::isnan(minCoeff) || std::isnan(maxCoeff)) {
      successStatus = JointCalibration::CalibStatus::FAIL_NUMERICAL;
    } else if (finalTestErrorAvg > imageSize.minCoeff() * reprojErrTolPercent ||
               finalErrorNorm > initialError.norm()) {
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
    result.reprojectionErrorNormCalib = finalErrorNorm;
    result.reprojectionErrorAvgCalib = finalErrorAvg;
    result.reprojectionErrorNormTest = finalTestErrorNorm;
    result.reprojectionErrorAvgTest = finalTestErrorAvg;
    result.sampleSizeTest = static_cast<size_t>(finalTestErrorVec.size());
    result.sampleSizeCalib = static_cast<size_t>(finalErrorVec.size());
#if DEBUG_SIM_TEST
    logStream << "END";
    std::lock_guard<std::mutex> lg(utils::mtx_cout_);
    std::cout << logStream.str() << std::endl;
#endif
    return {result, jointCalibResidual, initialError, finalErrorVec,
            finalTestErrorVec};
  }
};

} // namespace JointCalibEval
