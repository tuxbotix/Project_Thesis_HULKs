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
 * @brief evalJointErrorSet
 * @param naoModel
 * @param poseList
 * @param inducedErrorStdVec
 * @return pre-post residuals
 */
std::tuple<JointCalibSolvers::JointCalibResult, Eigen::VectorXf,
           Eigen::VectorXf, Eigen::VectorXf>
evalJointErrorSet(const NaoJointAndSensorModel naoModel,
                  const poseAndRawAngleListT &poseList,
                  const rawPoseT &inducedErrorStdVec, bool &success) {
  JointCalibSolvers::JointCalibResult result;

  const Eigen::VectorXf inducedErrorEig =
      Eigen::Map<const Eigen::VectorXf, Eigen::Unaligned>(
          inducedErrorStdVec.data(), inducedErrorStdVec.size());

  Eigen::VectorXf jointCalibResidual = inducedErrorEig;

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
        auto correspondances =
            NaoSensorDataProvider::getFilteredCorrespondancePairs(
                groundGrid,
                naoJointSensorModel.robotToPixelMulti(camName, groundGrid));
        if (correspondances.size() > 0) {
          frameCaptures.emplace_back(correspondances, poseAndAngles.angles,
                                     camName, poseAndAngles.pose.supportFoot);
        } else {
          std::cout << "No suitable ground points" << std::endl;
        }
      } else {
        std::cerr << "Obtaining ground grid failed" << std::endl;
      }
    }
  }

  if (frameCaptures.size() <= 0) {
    std::cout << "No suitable frame captures !!" << std::endl;
    success = false;
    return {result, jointCalibResidual, Eigen::VectorXf(0), Eigen::VectorXf(0)};
  }

  /*
   * Mini eval of cost fcn
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
   * Run Lev-Mar
   */
  Eigen::NumericalDiff<JointCalibSolvers::JointCalibrator> functorDiff(
      calibrator);
  Eigen::LevenbergMarquardt<
      Eigen::NumericalDiff<JointCalibSolvers::JointCalibrator>, float>
      lm(functorDiff);

  // Setting zero is very important.
  Eigen::VectorXf calibratedParams((int)JOINTS::JOINT::JOINTS_MAX);
  calibratedParams.setZero();

  int status = lm.minimize(calibratedParams);
  //  Get reprojection error
  calibrator(calibratedParams, finalErrorVec);

  auto minCoeff = finalErrorVec.minCoeff();
  auto maxCoeff = finalErrorVec.maxCoeff();
  auto errorNorm = finalErrorVec.norm();
  auto errorAvg = errorVec.norm() / errorVec.size();

  if (std::isnan(minCoeff) || std::isnan(maxCoeff) ||
      errorNorm > errorVec.norm()) {
    //        std::cout << "Calibration failure" << errorVec.norm() << " " <<
    //        finalErrorVec.norm() << std::endl;
    success = false;
  } else {
    success = true;
  }
  jointCalibResidual -= calibratedParams;

  const float tolerance = 1.0f * TO_RAD;

//  if (std::abs(jointCalibResidual.maxCoeff()) > tolerance ||
//      std::abs(jointCalibResidual.minCoeff()) > tolerance) {
//    std::lock_guard<std::mutex> lg(utils::mtx_cout_);
//    std::cout << "\n"
//              << calibratedParams.transpose() / TO_RAD << " \n"
//              << inducedErrorEig.transpose() / TO_RAD << "\n" << status << " " << errorNorm << std::max(minCoeff, maxCoeff)
//              << std::endl;
//  }
  result.status = status;
  result.jointParams.resize(JOINTS::JOINT::JOINTS_MAX);
  for (int i = 0; i < JOINTS::JOINT::JOINTS_MAX; ++i) {
    result.jointParams[i] = calibratedParams(i);
  }
  result.reprojectionErrorNorm = errorNorm;
  result.reprojectionErrorNorm = errorAvg;

  return {result, jointCalibResidual, errorVec, finalErrorVec};
}

void threadedFcn(CalibEvalResiduals<double> &residuals,
                 const poseAndRawAngleListT &poseList,
                 const NaoJointAndSensorModel &naoJointSensorModel,
                 const std::vector<rawPoseT>::iterator jointErrItrBegin,
                 const std::vector<rawPoseT>::iterator jointErrItrEnd) {
  size_t badCases = 0;
  size_t counter = 0;
  for (auto errSetIter = jointErrItrBegin; errSetIter != jointErrItrEnd;
       ++errSetIter, ++counter) {
    auto &errorSet = *errSetIter;

    bool success = false;

    JointCalibSolvers::JointCalibResult result;
    Eigen::VectorXf preResiduals;
    Eigen::VectorXf postResiduals;
    Eigen::VectorXf jointCalibResiduals;
    std::tie(result, jointCalibResiduals, preResiduals, postResiduals) =
        evalJointErrorSet(naoJointSensorModel, poseList, errorSet, success);

    if (!success) {
      ++badCases;
      continue;
    }

    auto prePostResSize = static_cast<size_t>(preResiduals.size());
    auto curPreResSize = residuals.preX.size();

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
    jointCalibResiduals /= TO_RAD;
    for (long i = 0; i < JOINTS::JOINT::JOINTS_MAX; ++i) {
      //      residuals.jointResiduals[static_cast<size_t>(i)].push_back(
      //          jointCalibResiduals(i));
      residuals.jointResiduals[0].push_back(jointCalibResiduals(i));
    }

    //    if (counter % 1000) {
    //    }
  }
  //  std::cout<<
  //  return badCases;
}

int main(int argc, char *argv[]) {
  std::string inFileName((argc > 1 ? argv[1] : "out"));
  //    std::string inFileName((argc > 1 ? argv[1] : "out"));
  std::string confRoot((argc > 2 ? argv[2] : "../../nao/home/"));

  //    size_t usableThreads = 0;

  std::fstream inFile(inFileName);

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

  // TODO multithread later...
  //    for(auto & iStreamRef : inputPoseAndJointStreams){

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
   * Make dataset
   */

  // TODO check if uniform distribution is the right choice?
  std::default_random_engine generator;
  std::uniform_real_distribution<float> distribution(-5, 5);
  //  std::normal_distribution<float> distribution(0.0, 2);

  const size_t JOINT_ERR_LST_DESIRED_COUNT = 1000;

  std::set<rawPoseT> uniqueJointErrList;
  //    std::vector<rawPoseT> jointErrList(JOINT_ERR_LST_DESIRED_COUNT);
  for (size_t iter = 0; iter < JOINT_ERR_LST_DESIRED_COUNT; iter++) {
    rawPoseT elem = rawPoseT(JOINTS::JOINT::JOINTS_MAX, 0.0f);

//    elem[JOINTS::JOINT::HEAD_PITCH] = std::min(
//        0.0f, static_cast<float>(distribution(generator)) / 2 * TO_RAD);
//    elem[JOINTS::JOINT::HEAD_YAW] = distribution(generator) * TO_RAD;

    // todo make this dual leg supported..
    for (size_t i = JOINTS::JOINT::L_HIP_YAW_PITCH;
         i <= JOINTS::JOINT::L_ANKLE_ROLL; i++) {
      elem[i] = distribution(generator) * TO_RAD;
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

  // initialize histograms
  std::cout << " initializing histograms " << std::endl;

  // start calib runs
  size_t badCases = 0;

  std::cout << " Running calibrations" << std::endl;

  std::vector<CalibEvalResiduals<double>> calibResidualList(MAX_THREADS);

  std::vector<std::thread> threadList(MAX_THREADS);

  size_t threadingOffsets = JOINT_ERR_LST_COUNT / MAX_THREADS;

  for (size_t i = 0; i < MAX_THREADS; ++i) {
    std::vector<rawPoseT>::iterator begin =
        jointErrList.begin() + (threadingOffsets * i);
    std::vector<rawPoseT>::iterator end =
        (i + 1 >= MAX_THREADS)
            ? jointErrList.end()
            : jointErrList.begin() + threadingOffsets * (i + 1);

    std::cout << "starting thread: " << i << std::endl;

    threadList[i] = std::thread(threadedFcn, std::ref(calibResidualList[i]),
                                std::ref(poseList),
                                std::ref(naoJointSensorModel), begin, end);
  }

  //  threadedFcn(calibResidualList[0], poseList, naoJointSensorModel,
  //  jointErrList.begin(), jointErrList.end());

  CalibEvalResiduals<double> finalResidualSet;

  std::cout << "going to join" << std::endl;
  for (size_t i = 0; i < MAX_THREADS; ++i) {
    auto &thread = threadList[i];
    auto &residual = calibResidualList[i];
    if (thread.joinable()) {
      thread.join();
    }
    if (i != 0) {
      finalResidualSet.joinEvalResiduals(residual, finalResidualSet);
    }
  }

  std::cout << "Processing done" << std::endl;

  auto minMaxPostX = std::minmax_element(finalResidualSet.postX.begin(),
                                         finalResidualSet.postX.end());

  auto minMaxPostY = std::minmax_element(finalResidualSet.postY.begin(),
                                         finalResidualSet.postY.end());
  auto minMaxJointParams =
      std::minmax_element(finalResidualSet.jointResiduals[0].begin(),
                          finalResidualSet.jointResiduals[0].end());

  utils::SimpleHistogram<double> preXhist(3000, -1500, 1500);
  utils::SimpleHistogram<double> preYhist(3000, -1500, 1500);

  auto maxOfAll = std::max(std::abs(*minMaxPostX.first), *minMaxPostX.second);
  maxOfAll =
      std::max(std::max(std::abs(*minMaxPostY.first), *minMaxPostY.second),
               maxOfAll) *
      1.1;

  std::cout << "minmax" << maxOfAll << " " << std::endl;

  utils::SimpleHistogram<double> postXhist(3000, -maxOfAll, maxOfAll);
  utils::SimpleHistogram<double> postYhist(3000, -maxOfAll, maxOfAll);

  auto maxOfJointParamsAll = 50;

  utils::SimpleHistogram<double> postParamhist(3000, -maxOfJointParamsAll,
                                               maxOfJointParamsAll);

  for (auto &elem : calibResidualList) {
    preXhist.update(elem.preX);
    preYhist.update(elem.preY);
    postXhist.update(elem.postX);
    postYhist.update(elem.postY);

    for (auto &jointErr : elem.jointResiduals) {
      postParamhist.update(jointErr);
    }
  }

  //    const Eigen::VectorXf preResidualsX = Eigen::Map<const Eigen::VectorXf,
  //    Eigen::Unaligned>(preResidualsXY.first.data(),
  //    preResidualsXY.first.size());
  //    const Eigen::VectorXf preResidualsY = Eigen::Map<const Eigen::VectorXf,
  //    Eigen::Unaligned>(preResidualsXY.second.data(),
  //    preResidualsXY.second.size());

  //    {
  //        const Eigen::VectorXf postResidualsX = Eigen::Map<const
  //        Eigen::VectorXf, Eigen::Unaligned>(postResidualsXY.first.data(),
  //        postResidualsXY.first.size());
  //       float mean = postResidualsX.mean();
  //       VectorXf val = (postResidualsX -
  //       VectorXf::Ones(postResidualsX.size()) * mean);
  //       double std_dev = std::sqrt(val.dot(val) / (postResidualsX.size() -
  //       1));
  //       std::cout << "X mean" << mean << " std dev: "
  //                 << std_dev << " norm " << postResidualsX.norm() <<
  //                 std::endl;
  //    }
  //    {
  //        const Eigen::VectorXf postResidualsY = Eigen::Map<const
  //        Eigen::VectorXf, Eigen::Unaligned>(postResidualsXY.second.data(),
  //        postResidualsXY.second.size());
  //       float mean = postResidualsY.mean();
  //       VectorXf val = (postResidualsY -
  //       VectorXf::Ones(postResidualsY.size()) * mean);
  //       double std_dev = std::sqrt(val.dot(val) / (postResidualsY.size() -
  //       1));
  //       std::cout << "Y mean" << mean << " std dev: "
  //                 << std_dev << " norm " << postResidualsY.norm() <<
  //                 std::endl;
  //    }

  std::cout << "Min: "
            << std::min(*minMaxJointParams.first, *minMaxJointParams.second)
            << " max: "
            << std::max(*minMaxJointParams.first, *minMaxJointParams.second)
            << " Bad cases: " << badCases / (float)jointErrList.size()
            << std::endl;
  //    std::cout << postYhist << std::endl;

  // dump the data into the files.

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

  for (auto elem :
       {originalResidualDumpX, originalResidualDumpY, calibratedResidualDumpX,
        calibratedResidualDumpY, calibratedJointParamResidualDumpY}) {
    std::cout << "Start dump to : " << elem.second << std::endl;
    std::pair<double, double> bounds = elem.first.getPercentileBounds(0.25);
    std::cout << " 25% bounds: " << bounds.first << " " << bounds.second
              << std::endl;
    bounds = elem.first.getPercentileBounds(0.1);
    std::cout << " 10% bounds: " << bounds.first << " " << bounds.second
              << std::endl;
    bounds = elem.first.getPercentileBounds(0.05);
    std::cout << " 5% bounds: " << bounds.first << " " << bounds.second
              << std::endl;
    bounds = elem.first.getPercentileBounds(0.01);
    std::cout << "  1% bounds: " << bounds.first << " " << bounds.second
              << std::endl;
    bounds = elem.first.getPercentileBounds(0.001);
    std::cout << "0.1% bounds: " << bounds.first << " " << bounds.second
              << std::endl;

    std::fstream file(elem.second, std::ios::out);
    file << elem.first;
    file.close();
  }

  return 0;
}
