#include <algorithm>
#include <assert.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <thread>

#include <Data/CameraMatrix.hpp>
#include <Modules/Configuration/Configuration.h>
#include <Modules/Configuration/UnixSocketConfig.hpp>
#include <Modules/NaoProvider.h>

#include <Hardware/RobotInterface.hpp>

#include <cxxopts/include/cxxopts.hpp>

#include "Interactions.hpp"
#include "MiniConfigHandle.hpp"
#include "NaoPoseInfo.hpp"
#include "ObservationSensitivityProvider.hpp"
#include "TUHHMin.hpp"
#include "constants.hpp"

#define DO_COMMIT 1
#define WRITE_PARALLEL 1
#define DEBUG_FILE_COMMIT 1
#define DEBUG_IN_THREAD 1
#define ENABLE_PROGRESS 1

#include "utils.hpp"

static const size_t MAX_MEM_FOR_PIP =
    20E9; // max mmory for pose vs pose interactions

const unsigned int MAX_THREADS = std::thread::hardware_concurrency();
static const int SENSOR_COUNT = 2;

using JointWeights = std::array<float, JOINTS::JOINT::JOINTS_MAX>;
using SensorWeights = std::array<float, SENSOR_COUNT>;
using SensMagAndPoseId =
    std::pair<int, size_t>; // Sensor Name should be maintained externally.

/**
 *
 */
using namespace Interactions;

const size_t maxPoseInteractionCount =
    MAX_MEM_FOR_PIP / sizeof(PoseInteraction);

enum SimilarityCostPolicy {
  NONE,
  PENALIZE_0_ONLY,
  PENALIZE_0_AND_180 // scaled to fit to 0 - 180 range.
};
const std::map<std::string, SimilarityCostPolicy> SimilarityCostPolicyMap{
    {"NONE", NONE},
    {"PENALIZE_0_ONLY", PENALIZE_0_ONLY},
    {"PENALIZE_0_AND_180", PENALIZE_0_AND_180}};

/**
 * Cost function for ranking a given pose for a given sensor.
 * Factors:
 * 1. Sensitivity magnitude
 * 2. Mutual orthogonality
 * Weights:
 * 1. w_sensor_mag - can be used to prioritize a given sensor's magnitudes
 * 2. w_joint - can be used to prioritize a given joint (esp. useful when
 * general optimization only favor some joints)
 * 3. w_sensor_ortho - Orthogonality weight for a given sensor.
 *
 * w_sensor_mag * w_joint and w_sensor_ortho * w_joint are used for magnitude
 * and orthogonality weighting respectively.
 */
template <typename T>
std::pair<double, InteractionCost> poseFilterCostFunc(
    const PoseSensitivity<T> &p, const float &wSensorMag,
    const JointWeights &wJoints, const float &wOrthoGeneric,
    const float &observableJointCountWeight,
    const SimilarityCostPolicy &policy = SimilarityCostPolicy::PENALIZE_0_ONLY,
    const bool &enableInteractionCostWeighting = true) {

  InteractionCost interactionCost;
  interactionCost.setZero();
  double accum =
      static_cast<double>(observableJointCountWeight) * p.getObservableCount();
  if (std::isnan(accum) || p.getObservableCount() <= 0) {
    std::lock_guard<std::mutex> lock(utils::mtx_cout_);
    std::cout << p.getId() << " is NAN 0 stage || obsCount = 0" << std::endl;
    return {10E10, interactionCost};
  }
  /*
   * We'll go like a triangle, as pj.pk = pk.pj, only n*(n+1)/2 iterations
   * needed. See Interactions::encodeLowerTriangle...
   */

  for (size_t row = 1; row < TOT_OBS_JOINT_COUNT; row++) {
    const JOINTS::JOINT joint = Sensor::ALL_OBS_JOINTS[row];
    if (!p.isJointObservable(joint)) {
      continue;
    }
    const double wMag = static_cast<double>(wSensorMag * wJoints[joint]);
    const double wOrtho = static_cast<double>(wOrthoGeneric * wJoints[joint]);
    T val;
    bool obs;
    p.getSensitivity(joint, val, obs);
    // stage 1
    accum += wMag * static_cast<double>(val.norm());
    if (std::isnan(val.norm()) || std::isnan(accum)) {
      std::lock_guard<std::mutex> lock(utils::mtx_cout_);
      std::cout << p.getId() << " is NAN 1st stage joint: " << joint << " val "
                << val.norm() << std::endl;
      continue;
    }

    double v1Norm = static_cast<double>(val.norm());
    for (size_t col = 0; col < row; col++) {
      const JOINTS::JOINT innerJoint = Sensor::ALL_OBS_JOINTS[col];
      // skip own-joint and unobservable joints..
      if (!p.isJointObservable(innerJoint)) {
        continue;
      }
      T val2;
      bool obs2;
      p.getSensitivity(innerJoint, val2, obs2);

      double v2Norm = static_cast<double>(val2.norm());
      // second stage
      double dot =
          std::fabs(static_cast<double>(val.dot(val2)) / (v1Norm * v2Norm));
      // We trust in the dot product :P
      if (dot > 1.0) {
        dot = 1.0;
      } else if (dot < -1.0) {
        dot = -1.0;
      }
      // get the linear angle value.. closer to 90deg, the better.
      const double similarityAngle = std::acos(dot) / TO_RAD_DBL;
      const long curIdx = encodeInteractionVecIndex<Eigen::Index>(row, col);

      /*
       * Below are a few policies..
       * bets means cheapest (lowest cost)
       * TODO investigate how would not penalizing 180 goes on.
       */
      if (policy == SimilarityCostPolicy::PENALIZE_0_ONLY) {
        /// [0, 180] 180 is best, 90 is better and 0 is worst
        interactionCost(curIdx) = std::fabs(180.0 - similarityAngle);
      } else if (policy == SimilarityCostPolicy::PENALIZE_0_AND_180) {
        /// [0, 90*2] 90 is best, 180 (wrapped to 0) and 0 are worst
        interactionCost(curIdx) = std::fabs(90.0 - similarityAngle) * 2;
      } else {
        /// [0, 180] Original - wrong outcome;
        interactionCost(curIdx) = std::fabs(similarityAngle);
      }

      // we accumilate the "fitness" or negative cost; so the negative sign
      accum += wOrtho * -interactionCost(curIdx);

      if (enableInteractionCostWeighting) {
        interactionCost(curIdx) *= wOrtho;
      }

      if (dot > static_cast<double>(1.0) || dot < static_cast<double>(-1.0) ||
          std::isnan(dot) || std::isnan(similarityAngle) || std::isnan(accum)) {
        std::lock_guard<std::mutex> lock(utils::mtx_cout_);
        std::cout << p.getId() << " is NAN 2nd stage [domain err] " << joint
                  << " " << innerJoint << " dot " << dot << " aco "
                  << std::acos(dot) << " wOrtho " << wOrtho << " fabs "
                  << std::fabs(similarityAngle) << std::endl;
        continue;
      }
    }
  }
  return {-accum, interactionCost};
}

/**
 * This runs the cost function :)
 * @param poseCosts list of all cost minimas (local minimas).
 */
void poseFilterFunc(
    std::istream &inputStream, std::vector<PoseCost> &poseCosts,
    const SensorWeights sensorMagnitudeWeights, const JointWeights jointWeights,
    const float orthogonalityWeight, const float observableJointCountWeight,
    std::atomic<size_t> &iterations,
    const SimilarityCostPolicy policy = SimilarityCostPolicy::PENALIZE_0_ONLY,
    const bool enableInteractionCostWeighting = true) {
  bool direction = false;
  PoseCost previousCostVal(100, 0);
  PoseCost curCostVal(100, 0);

  bool first = true;

  PoseSensitivity<Vector3f> p;
  size_t sensorAsIndex;

  while (utils::getNextDataEntry<PoseSensitivity<Vector3f>>(inputStream, p)) {
    iterations++;
    // if nothing to observe, skip :P
    // TODO penalize poses that aren't much observable by sensors?
    if (p.getObservableCount() <= 0) {
      continue;
    }
    size_t tempId = p.getId();

    // Things don't match up = new pose loaded, thus commit the previous cost.
    if (first || curCostVal.id != tempId) {
      curCostVal.interactionCount = 0;
      for (Eigen::Index i = 0;
           i < static_cast<Eigen::Index>(TOT_AMBIGUITY_COMBOS); ++i) {
        if (std::abs(curCostVal.jointInteractionCostVec(i)) > __DBL_EPSILON__) {
          curCostVal.interactionCount++;
        }
      }
      // current pose's total cost is lesser than previous pose's total cost
      bool curDir = (curCostVal.jointCost < previousCostVal.jointCost);
      // falling edge - at a peak, but if first dont commit!
      if (!curDir && direction && !first) {
        poseCosts.push_back(curCostVal);
      }

      previousCostVal = curCostVal;
      curCostVal.reset();
      curCostVal.id = tempId;
      direction = curDir;
      // std::cout << "new sensor" << std::endl;
      first = false;
    }
    sensorAsIndex = static_cast<size_t>(p.getSensorName());
    auto costInfos = poseFilterCostFunc<Vector3f>(
        p, sensorMagnitudeWeights[sensorAsIndex], jointWeights,
        orthogonalityWeight, observableJointCountWeight, policy,
        enableInteractionCostWeighting);

    curCostVal.jointCost += costInfos.first;
    curCostVal.jointInteractionCostVec += costInfos.second;
  }
}

/**
 * @brief sortPosesWithCostFunc
 * @param usableThreads
 * @param inputPoseAndJointStreams
 * @param sensorMagnitudeWeights
 * @param jointWeights
 * @param wOrthoGeneric
 * @param observableJointCountWeight
 * @param poseCosts
 */
void filterAndSortPosesThreaded(
    const size_t &usableThreads,
    std::vector<std::fstream> &inputPoseAndJointStreams,
    const SensorWeights &sensorMagnitudeWeights,
    const JointWeights &jointWeights, const float &wOrthoGeneric,
    const float &observableJointCountWeight, std::vector<PoseCost> &poseCosts,
    const SimilarityCostPolicy &policy = SimilarityCostPolicy::PENALIZE_0_ONLY,
    const bool &enableInteractionCostWeighting = true) {

  std::vector<std::thread> threadList(usableThreads);
  std::vector<std::atomic<size_t>> iterCount(usableThreads);

  // threads -> sensors -> joints -> peakList (PerJointPerSensorPerThread)
  std::vector<std::vector<PoseCost>> poseCostListList(usableThreads);
  // launch the threads
  for (unsigned int i = 0; i < usableThreads; i++) {
    threadList[i] = std::thread(
        poseFilterFunc, std::ref(inputPoseAndJointStreams[i]),
        // std::ref(outputFileList[i]),
        std::ref(poseCostListList[i]), sensorMagnitudeWeights, jointWeights,
        wOrthoGeneric, observableJointCountWeight, std::ref(iterCount[i]),
        policy, enableInteractionCostWeighting);
  }

#if ENABLE_PROGRESS
  /*
   * Progress tracking enabled
   */
  bool continueTicker = true;
  std::thread tTick = std::thread([&]() {
    int elapsed = 0;
    const size_t interval = 5;
    while (continueTicker) {
      size_t iterSum = std::accumulate(iterCount.begin(), iterCount.end(),
                                       static_cast<size_t>(0));
      if (elapsed % 100) {
        std::lock_guard<std::mutex> lock(utils::mtx_cout_);
        std::cout << "Elapsed: " << elapsed
                  << "s Iterations: " << std::scientific
                  << static_cast<double>(iterSum) << std::endl;
      } else {
        std::stringstream t;
        t << "Elapsed: " << elapsed << " ";
        for (size_t i = 0; i < usableThreads; i++) {
          t << "T" << i << " :" << iterCount[i].load() << " ";
        }
        {
          std::lock_guard<std::mutex> lock(utils::mtx_cout_);
          std::cout << t.str() << std::endl;
        }
      }
      elapsed += interval;
      std::this_thread::sleep_for(
          std::chrono::seconds(interval)); // 100Hz -> 10ms
    }
  });
#endif
  /*
   * Join the worker threads
   */
  for (auto &t : threadList) {
    if (t.joinable()) {
      t.join();
    }
  }
#if ENABLE_PROGRESS
  // Join the progress thread
  continueTicker = false;
  if (tTick.joinable()) {
    tTick.join();
  }
#endif

  // first join poseCost vectors
  std::cout << poseCostListList[0].size() << " ";
  for (size_t t = 0; t < usableThreads; t++) // per thread
  {
    std::cout << poseCostListList[t].size() << " ";
    poseCosts.insert(poseCosts.end(),
                     std::make_move_iterator(poseCostListList[t].begin()),
                     std::make_move_iterator(poseCostListList[t].end()));
  }
  std::cout << std::endl;
}

int main(int argc, char **argv) {

  //    auto idx = encodeLowerTriangleIndex(10, 5);
  //    auto p = decodeLowerTriangleIndex(idx);

  //    std::cout << idx << " " << p.first << " " << p.second << std::endl;
  //    return 1;
  std::string inFileName((argc > 1 ? argv[1] : "out"));
  std::string favouredJoint((argc > 2 ? argv[2] : "-1"));
  std::string favouredSensor((argc > 3 ? argv[3] : "all")); // t, b, all
  std::string chainingModeStr(
      (argc > 4 ? argv[4] : "n")); // n, s, m - chaining modes.
  std::string confRoot((argc > 5 ? argv[5] : "../../nao/home/"));

  int jointNum = std::stoi(favouredJoint);
  if (jointNum < 0 || jointNum >= static_cast<int>(JOINTS::JOINT::JOINTS_MAX)) {
    jointNum = -1;
  }

  // 1= sum, 2 = multi
  int chainingMode = 0;
  if (chainingModeStr.compare("s") == 0) {
    chainingMode = 1;
  } else if (chainingModeStr.compare("m") == 0) {
    chainingMode = 2;
  }

  /*
   * Get config values.
   */
  Uni::Value confValue;
  if (!MiniConfigHandle::mountFile("configuration/poseFilterConf.json",
                                   confValue)) {
    std::cout << "couldn't open the conf file" << std::endl;
    exit(1);
  }

  std::string policyStr = confValue["currentPolicy"].asString();
  SimilarityCostPolicy policy = SimilarityCostPolicyMap.at(policyStr);
  bool enableInteractionCostWeighting =
      confValue["enableInteractionCostWeighting"].asBool();

  Uni::Value nearestNeighbourBounds = confValue["nearestNeighbourBounds"];
  HeadYawPitch hYPbounds(
      static_cast<float>(
          nearestNeighbourBounds["headYawPitch"].at(0).asDouble()),
      static_cast<float>(
          nearestNeighbourBounds["headYawPitch"].at(1).asDouble()));
  Vector3f torsoPosBound;
  Vector3f torsoRotBound;
  nearestNeighbourBounds["torsoPos"] >> torsoPosBound;
  nearestNeighbourBounds["torsoRot"] >> torsoRotBound;
  /*
   * Print input params
   */
  std::cout << "Favoured Joint:\t "
            << (favouredJoint == "-1" ? "Generic" : favouredJoint)
            << "\nFavoured sensor:\t"
            << (favouredSensor == "all" ? "All" : favouredSensor)
            << "\nFilter policy:\t" << policyStr << "\nInterCostWeight:\t"
            << enableInteractionCostWeighting << std::endl;

  const std::string outCostsFileName(
      inFileName + "_" + constants::FilteredPoseCostsFileName + "_" +
      (jointNum >= 0 ? "j" + std::to_string(jointNum) : "generic"));
  const std::string outFilteredPosesFileName(
      inFileName + "_" + constants::FilteredPosesFileName + "_" +
      (jointNum >= 0 ? "j" + std::to_string(jointNum) : "generic"));

  TUHH tuhhInstance(confRoot);

  std::cout << "init" << std::endl;

  size_t usableThreads = 0;

  std::vector<std::fstream> inputPoseAndJointStreams;
  for (size_t i = 0; i < MAX_THREADS; i++) {
    std::string fileName = inFileName + "_" +
                           constants::ExtractedSensitivitiesFileName + "_" +
                           std::to_string(i) + ".txt";
    if (std::ifstream(fileName)) {
      inputPoseAndJointStreams.emplace_back(fileName, std::ios::in);
      usableThreads++;
    } else {
      break;
    }
  }
  std::cout << "make outfiles. Usable threads -> " << usableThreads
            << std::endl;
  std::fstream outCostsFile =
      std::fstream((outCostsFileName + ".txt"), std::ios::out);
  std::fstream outFilteredPosesFile =
      std::fstream((outFilteredPosesFileName + ".txt"), std::ios::out);

  if (!outCostsFile.is_open() || !outFilteredPosesFile.is_open()) {
    std::cerr << "output file creation failed. Aborting!!!" << std::endl;
    exit(1);
  }

  /// Start the real threading..

  std::vector<std::thread> threadList(usableThreads);
  std::vector<std::atomic<size_t>> iterCount(usableThreads);

  // threads -> sensors -> joints -> peakList (PerJointPerSensorPerThread)
  std::vector<std::vector<PoseCost>> poseCostListList(usableThreads);
  SensorWeights sensorMagnitudeWeights;
  sensorMagnitudeWeights.fill(2);

  if (favouredSensor.compare("t") == 0) {
    sensorMagnitudeWeights[SENSOR_NAME::TOP_CAMERA] = 4;
  } else if (favouredSensor.compare("b") == 0) {
    sensorMagnitudeWeights[SENSOR_NAME::BOTTOM_CAMERA] = 4;
  } else if (favouredSensor.compare("all") == 0) {
    sensorMagnitudeWeights[SENSOR_NAME::TOP_CAMERA] = 4;
    sensorMagnitudeWeights[SENSOR_NAME::BOTTOM_CAMERA] = 4;
  }

  // TODO change this as needed.
  JointWeights jointWeights;
  jointWeights.fill(1);
  if (jointNum > 0 && jointNum < static_cast<int>(JOINTS::JOINT::JOINTS_MAX)) {
    jointWeights[static_cast<size_t>(jointNum)] = 4;
  }

  const float wOrthoGeneric = 3;
  const float observableJointCountWeight = 1;

  // Sorted PoseCost vector
  std::vector<PoseCost> sortedPoseCostVec;
  /*
   * send the input streams and get the sorted poses
   */
  filterAndSortPosesThreaded(
      usableThreads, inputPoseAndJointStreams, sensorMagnitudeWeights,
      jointWeights, wOrthoGeneric, observableJointCountWeight,
      sortedPoseCostVec, policy, enableInteractionCostWeighting);
  /*
     * Start mapping pose cost to poses
     */
  std::map<size_t, poseAndRawAngleT> poseMap;
  mapPoseCostToPoses(poseMap, inFileName, sortedPoseCostVec, usableThreads);

  /*
   * Sort the pose costs..
   */

  // if fails, we have a problem
  if (!sortAndTrimPoseCosts(poseMap, sortedPoseCostVec, hYPbounds,
                            torsoPosBound, torsoRotBound)) {
    return 1;
  }
  {
    for (size_t iter = 0; iter < 20; ++iter) {
      auto &poseCost = sortedPoseCostVec[iter];

      std::cout << poseCost.id << " InteractionCosts: ";
      for (Eigen::Index i = 0;
           i < static_cast<Eigen::Index>(TOT_AMBIGUITY_COMBOS); ++i) {
        if (std::abs(poseCost.jointInteractionCostVec(i)) > __DBL_EPSILON__) {
          auto idxPair = decodeInteractionVecIndex(i);
          std::cout << i << " (" << Sensor::ALL_OBS_JOINTS[idxPair.first]
                    << " vs " << Sensor::ALL_OBS_JOINTS[idxPair.second]
                    << "): " << static_cast<int>(std::round(
                                    poseCost.jointInteractionCostVec(i)))
                    << ", ";
        }
      }
      std::cout << std::endl;
    }
  }
  /*
   * Secondary sort run with poseInteractionns
   */
  std::vector<PoseInteraction> poseInteractionList;
  size_t approxMaxPosesToTry = populatePoseInteractions(
      poseInteractionList, sortedPoseCostVec, maxPoseInteractionCount);
  poseToPoseInteractionSort(poseInteractionList);

  // populate Id vec
  PoseInteraction rootPoseInteraction =
      poseInteractionList[0]; // this is the best
  std::vector<size_t> poseIdVec;
  {
    /*
     * this keeps the poses as ordered before. But being a set, we ensure only a
     * unique set of elems are here.
     */
    std::unordered_set<size_t> poseIdSet;
    poseIdVec.reserve(approxMaxPosesToTry); // reserve the vec
    poseIdSet.reserve(approxMaxPosesToTry); // reserve the set
    for (auto &elem : poseInteractionList) {
      auto res = poseIdSet.insert(elem.combinedId.first);
      if (res.second) {
        poseIdVec.emplace_back(elem.combinedId.first);
      }
      res = poseIdSet.insert(elem.combinedId.second);
      if (res.second) {
        poseIdVec.emplace_back(elem.combinedId.second);
      }
    }

    if (poseIdVec[0] == rootPoseInteraction.combinedId.first &&
        poseIdVec[1] == rootPoseInteraction.combinedId.second) {
      std::cout << "good" << std::endl;
    }
    poseInteractionList.clear(); // clear elements
    std::vector<PoseInteraction>().swap(
        poseInteractionList); // Actually force it to dealloc the memory
  }

  /*
   * Chain the poses!!
   */
  if (chainingMode != 0) {
    bool multiplicationMode = (chainingMode == 2);
    std::cout << "Start "
              << (multiplicationMode ? "Multiplication" : "Summation")
              << " Chaining" << std::endl;

    if (!poseToPoseChaining(rootPoseInteraction, poseIdVec, sortedPoseCostVec,
                            multiplicationMode, 20, MAX_THREADS)) {
      std::cerr << "Something went wrong in pose chaining" << std::endl;
      return 1;
    }
  }
  /*
   * Now write to file..
   */
  {
/// Write the remaining buffers to file
#if DO_COMMIT
    std::cout << "flushing all " << std::endl;
    std::cout << "Commit to file.." << poseIdVec.size() << std::endl;
    size_t iter = 0;
    for (auto &id : poseIdVec) {
      auto pose = poseMap.at(id);
      try {
        auto result =
            std::find_if(sortedPoseCostVec.begin(), sortedPoseCostVec.end(),
                         [&id](PoseCost &c) { return c.id == id; });
        if (result != sortedPoseCostVec.end()) {
          //                        poseList[i].pose.
          outFilteredPosesFile << pose << "\n";
          outCostsFile << "poseCost " << result->id << " " << result->jointCost
                       << " " << result->jointInteractionCostVec.transpose()
                       << "\n";
          iter++;
        }
      } catch (std::exception e) {
        std::cerr << e.what() << std::endl;
        return 1;
      }
    }
    outCostsFile.flush();
    outFilteredPosesFile.flush();
    std::cout << "commited % "
              << (iter * 100) / static_cast<double>(sortedPoseCostVec.size())
              << std::endl;
    // outputFileList[t].close();
    outCostsFile.close();
    outFilteredPosesFile.close();
#endif
    std::cout << "All done\n" << std::endl;
  }
  return 0;
}
