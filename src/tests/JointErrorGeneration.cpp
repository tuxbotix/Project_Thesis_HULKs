#include <array>
#include <fstream>
#include <iostream>
#include <iterator>
#include <random>
#include <unordered_set>

#include <cxxopts/include/cxxopts.hpp>

#define DEBUG_JOINT_AND_SENS 0

#include "constants.hpp"

#define DEBUG_SIM_TEST 0

#include "MiniConfigHandle.hpp"
#include "NaoPoseInfo.hpp"

#define DO_COMMIT 1
#define WRITE_PARALLEL 1
#define DEBUG_FILE_COMMIT 1
#define DEBUG_IN_THREAD 1
#define ENABLE_PROGRESS 1

#include "utils.hpp"

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
      "c,confRoot", "Conf path",
      cxxopts::value<std::string>()->default_value("../../nao/home/"))(
      "s,supportFoot", "Support foot, l, r or d",
      cxxopts::value<std::string>()->default_value("l"))(
      "h,head", "head joints",
      cxxopts::value<std::string>()->default_value("y"));

  auto result = options.parse(argc, argv);

  const std::string outFileName = result["file-prefix"].as<std::string>();
  //  const size_t MAX_POSES_TO_CALIB = result["n-poses"].as<size_t>();
  // Default is false
  std::string confRoot = result["confRoot"].as<std::string>();
  bool headJoints = result["head"].as<std::string>().compare("y") == 0;

  std::string supportFootName = result["supportFoot"].as<std::string>();

  SUPPORT_FOOT supportFoot = SUPPORT_FOOT::SF_DOUBLE;
  if (supportFootName.compare("l") == 0) {
    supportFoot = SUPPORT_FOOT::SF_LEFT;
  } else if (supportFootName.compare("r") == 0) {
    supportFoot = SUPPORT_FOOT::SF_RIGHT;
  } else {
    std::cout << "Going for double foot." << std::endl;
    supportFootName = "d";
  }

  std::fstream outFile(outFileName, std::ios::out);
  if (!outFile.is_open()) {
    std::cerr << "output file '" << outFileName
              << "'cannot be opened, exiting.." << std::endl;
    return 1;
  }

  /// Get config values.
  Uni::Value confValue;
  if (!MiniConfigHandle::mountFile("configuration/simGridEvalConf.json",
                                   confValue)) {
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

  /*
   * Print input params
   */
  std::cout << "Errors to try:\t" << JOINT_ERR_LST_DESIRED_COUNT
            << "\nMinMax Error val:\t" << MIN_ERR_VAL << ", " << MAX_ERR_VAL
            << "\nJoint Calib Tol (deg) :\t" << JOINT_CALIB_QUALITY_TOL_DEG
            << std::endl;

  /*
   * Make dataset (joint errors)
   */

  // TODO check if uniform distribution is the right choice?
  std::default_random_engine generator;
  /// New change, try 1 to 6, -1 to -6 only
  std::uniform_real_distribution<float> distribution(MIN_ERR_VAL, MAX_ERR_VAL);

  //  std::uniform_int_distribution<int> binDistribution(0, 1);
  //  const auto plusOrMinus = [&binDistribution, &generator]() -> float {
  //    return binDistribution(generator) ? 1.0f : -1.0f;
  //  };
  //  std::normal_distribution<float> distribution(0.0, 2);

  std::set<rawPoseT> uniqueJointErrList;

  for (size_t iter = 0; iter < JOINT_ERR_LST_DESIRED_COUNT; iter++) {
    rawPoseT elem = rawPoseT(JOINTS::JOINT::JOINTS_MAX, 0.0f);

    /// Do head angles seperately..
    if (headJoints) {
      elem[JOINTS::JOINT::HEAD_PITCH] =
          std::min(0.0f, static_cast<float>(distribution(generator)) / 2.0f *
                             TO_RAD_FLT);
      elem[JOINTS::JOINT::HEAD_YAW] =
          distribution(generator) * TO_RAD_FLT /* * plusOrMinus() */;
    }
    /// Leg Angles
    if (supportFoot == SUPPORT_FOOT::SF_DOUBLE ||
        supportFoot == SUPPORT_FOOT::SF_LEFT) {
      for (size_t i = JOINTS::JOINT::L_HIP_YAW_PITCH;
           i <= JOINTS::JOINT::L_ANKLE_ROLL; i++) {
        elem[i] = distribution(generator) * TO_RAD_FLT /* * plusOrMinus()*/;
      }
    }
    if (supportFoot == SUPPORT_FOOT::SF_DOUBLE ||
        supportFoot == SUPPORT_FOOT::SF_RIGHT) {
      for (size_t i = JOINTS::JOINT::R_HIP_YAW_PITCH;
           i <= JOINTS::JOINT::R_ANKLE_ROLL; i++) {
        elem[i] = distribution(generator) * TO_RAD_FLT /* * plusOrMinus()*/;
      }
    }
    uniqueJointErrList.insert(elem);
  }

  std::cout << uniqueJointErrList.size() << "error lists to try" << std::endl;

  for (auto &errorConfig : uniqueJointErrList) {
    for (auto &angle : errorConfig) {
      outFile << angle << " ";
    }
    outFile << "\n";
  }
  outFile.flush();
  outFile.close();
  std::cout << "finish committing to file: " << outFileName << std::endl;
}
