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
#include <Modules/Poses.h>

#include <Hardware/RobotInterface.hpp>
#include <Tools/Kinematics/KinematicMatrix.h>
#include <Tools/Storage/Image.hpp>
#include <Tools/Storage/Image422.hpp>

#include "NaoPoseInfo.hpp"
#include "NaoStability.hpp"
#include "NaoTorsoPose.hpp"
#include "TUHHMin.hpp"
#include "constants.hpp"
// #define DEBUG_CAM_OBS 1
#include "JointCalibSolver.hpp"
#include "ObservationSensitivityProvider.hpp"

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

using PoseMap = std::map<size_t, poseAndRawAngleT>;

using JointWeights = std::array<float, JOINTS::JOINT::JOINTS_MAX>;
using SensorWeights = std::array<float, SENSOR_COUNT>;
using SensMagAndPoseId =
    std::pair<int, size_t>; // Sensor Name should be maintained externally.

const size_t TOT_OBS_JOINT_COUNT =
    JointCalibSolvers::COMPACT_JOINT_CALIB_PARAM_COUNT;
const size_t TOT_AMBIGUITY_COMBOS =
    TOT_OBS_JOINT_COUNT * (TOT_OBS_JOINT_COUNT - 1) / 2;

using InteractionCost = Eigen::Matrix<int, TOT_AMBIGUITY_COMBOS, 1>;
using PoseInteractionId = std::pair<size_t, size_t>; // id1, id2 in order

struct PoseCost {
  double jointCost;
  // this accumilate total costs found based on below pose interaction cost, in
  // practice, this only holds the costs encountered so far while making
  // poseInteractions
  double poseInteractionCost;
  size_t id;
  InteractionCost
      jointInteractionCostVec; // j1 vs j2, j1 vs j3, ... for all joint
                               // configs under calibration
  int interactionCount;        // count of non-zero elements of interaction cost

  PoseCost(const double &cost, const size_t &id)
      : jointCost(cost), poseInteractionCost(0), id(id), interactionCount(0) {
    jointInteractionCostVec.setZero();
  }
  void reset() {
    jointCost = 0;
    poseInteractionCost = 0;
    id = 0;
    jointInteractionCostVec.setZero();
    interactionCount = 0;
  }
};

inline PoseInteractionId makeCombinedId(const size_t &id1, const size_t &id2) {
  return (id1 < id2) ? PoseInteractionId{id1, id2}
                     : PoseInteractionId{id2, id1};
}

struct PoseInteraction {
  PoseInteractionId combinedId; // id1, id2 in order
  double cost;
  int interactionCount; // count of non-zero elements of interaction cost
  // In this case, my approach is elementwise multiplication
  //  InteractionCost
  //      poseInteractionCostVec; // p1 vs p2, p1 vs p3, ... for all joint
  // configs under calibration
  PoseInteraction(const PoseCost &c1, const PoseCost &c2) {

    combinedId = (c1.id < c2.id) ? PoseInteractionId{c1.id, c2.id}
                                 : PoseInteractionId{c2.id, c1.id};

    //    interactionCost = (c1.interactionCost - c2.interactionCost).
    InteractionCost
        poseInteractionCostVec; // p1 vs p2, p1 vs p3, ... for all joint
    for (Eigen::Index i = 0;
         i < static_cast<Eigen::Index>(TOT_AMBIGUITY_COMBOS); ++i) {
      poseInteractionCostVec(i) =
          c1.jointInteractionCostVec(i) * c2.jointInteractionCostVec(i);
      if (poseInteractionCostVec(i) > 0) {
        interactionCount++;
      }
    }
    cost = poseInteractionCostVec.sum();
  }

  inline static void
  getInteractionVecAndCount(InteractionCost &interactionCostVec,
                            int &interactionCount, const PoseCost &c1,
                            const PoseCost &c2) {
    for (Eigen::Index i = 0;
         i < static_cast<Eigen::Index>(TOT_AMBIGUITY_COMBOS); ++i) {
      interactionCostVec(i) =
          c1.jointInteractionCostVec(i) * c2.jointInteractionCostVec(i);
      if (interactionCostVec(i) > 0) {
        interactionCount++;
      }
    }
  }
};

const size_t maxPoseInteractionCount =
    MAX_MEM_FOR_PIP / sizeof(PoseInteraction);

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
std::pair<double, InteractionCost>
poseFilterCostFunc(const PoseSensitivity<T> &p, const float &wSensorMag,
                   const JointWeights &wJoints, const float &wOrthoGeneric,
                   const float &observableJointCountWeight) {
  InteractionCost interactionCost;
  interactionCost.setZero();
  double accum =
      static_cast<double>(observableJointCountWeight) * p.getObservableCount();
  if (std::isnan(accum)) {
    std::lock_guard<std::mutex> lock(utils::mtx_cout_);
    std::cout << p.getId() << " is NAN 0 stage" << std::endl;
    return {std::numeric_limits<double>::max(), interactionCost};
  }
  /*
   * We'll go like a triangle, as pj.pk = pk.pj, only n*(n+1)/2 iterations
   * needed
   * ex:
   * p1p2 p1p3 p1p4
   * p2p3 p2p4
   * p3p4
   */

  for (size_t j = 0; j < TOT_OBS_JOINT_COUNT - 1; j++) {
    const JOINTS::JOINT joint = static_cast<JOINTS::JOINT>(j);
    if (!p.isJointObservable(joint)) {
      continue;
    }
    const double wMag = static_cast<double>(wSensorMag * wJoints[j]);
    const double wOrtho = static_cast<double>(wOrthoGeneric * wJoints[j]);
    T val;
    bool obs;
    p.getSensitivity(joint, val, obs);
    // stage 1
    accum += wMag * static_cast<double>(val.norm());
    if (std::isnan(val.norm()) || std::isnan(accum)) {
      std::lock_guard<std::mutex> lock(utils::mtx_cout_);
      std::cout << p.getId() << " is NAN 1st stage joint: " << j << " val "
                << val.norm() << std::endl;
      continue;
    }

    double v1Norm = static_cast<double>(val.norm());
    for (size_t k = j + 1; k < TOT_OBS_JOINT_COUNT; k++) {
      const JOINTS::JOINT innerJoint = static_cast<JOINTS::JOINT>(k);
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

      double similarityAngle = std::fabs(std::acos(dot) / TO_RAD_DBL);
      const long curIdx = utils::getIndexUpperTriangle<Eigen::Index>(
          j, k, TOT_AMBIGUITY_COMBOS);
      interactionCost(curIdx) =
          static_cast<int>(std::round(std::abs(180.0 - similarityAngle)));

      accum += wOrtho * similarityAngle;
      if (dot > static_cast<double>(1.0) || dot < static_cast<double>(-1.0) ||
          std::isnan(dot) || std::isnan(accum)) {
        std::lock_guard<std::mutex> lock(utils::mtx_cout_);
        std::cout << p.getId() << " is NAN 2nd stage [domain err] " << j << " "
                  << k << " dot " << dot << " aco " << std::acos(dot)
                  << " wOrtho " << wOrtho << " fabs "
                  << std::fabs(std::acos(dot) / TO_RAD_DBL) << std::endl;
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
void poseFilterFunc(std::istream &inputStream, std::vector<PoseCost> &poseCosts,
                    const SensorWeights sensorMagnitudeWeights,
                    const JointWeights jointWeights,
                    const float orthogonalityWeight,
                    const float observableJointCountWeight,
                    std::atomic<size_t> &iterations) {
  bool direction = false;
  PoseCost previousCostVal(100, 0);
  PoseCost curCostVal(100, 0);
  // Per pose, multiple sensors are there,..
  //  size_t curPoseId = 0;
  bool first = true;
  // if at least one sensor had observations at that pose. If not, drop that
  // pose from this level.
  // bool anySensorObserved = false;

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
        if (curCostVal.jointInteractionCostVec(i) != 0) {
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
        orthogonalityWeight, observableJointCountWeight);

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
    const float &observableJointCountWeight, std::vector<PoseCost> &poseCosts) {

  std::vector<std::thread> threadList(usableThreads);
  std::vector<std::atomic<size_t>> iterCount(usableThreads);

  // threads -> sensors -> joints -> peakList (PerJointPerSensorPerThread)
  std::vector<std::vector<PoseCost>> poseCostListList(usableThreads);
  // launch the threads
  for (unsigned int i = 0; i < usableThreads; i++) {

    //     void poseFilterFunc(std::istream &inputStream,
    //                         // std::ostream &outputStream,
    //                         std::vector<PoseCost> &poseCosts,
    //                         const SensorWeights sensorMagnitudeWeights,
    //                         const JointWeights jointWeights,
    //                         const float orthogonalityWeight,
    //                         std::atomic<size_t> &iterations, bool
    //                         lastPortion)

    threadList[i] = std::thread(
        poseFilterFunc, std::ref(inputPoseAndJointStreams[i]),
        // std::ref(outputFileList[i]),
        std::ref(poseCostListList[i]), sensorMagnitudeWeights, jointWeights,
        wOrthoGeneric, observableJointCountWeight, std::ref(iterCount[i]));
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

/**
 * @brief mapPoseCostToPoses
 * @param poseMap
 * @param inFileName
 * @param sortedPoseCostVec
 * @param usableThreads
 * @return
 */
bool mapPoseCostToPoses(PoseMap &poseMap, const std::string &inFileName,
                        const std::vector<PoseCost> &sortedPoseCostVec,
                        const size_t &usableThreads) {
  /*
   * Start mapping pose cost to poses
   */

  std::fstream genPoseListFile;

  size_t outFileNum = 0;
  std::cout << "opening " << outFileNum << std::endl;
  genPoseListFile.open(
      (inFileName + "_GeneratedPoses_" + std::to_string(outFileNum) + ".txt"),
      std::ios::in);

  // Read each generated pose file and match with the posecost vector.
  for (size_t idx = 0; idx < sortedPoseCostVec.size(); idx++) // per thread
  {
    auto &elem = sortedPoseCostVec[idx];

    std::string curString = "";
    std::string dummy = "";
    size_t id = 0;
    bool elementFound = false;

    poseAndRawAngleT tempPose;

    while (std::getline(genPoseListFile, curString)) {
      std::stringstream curStream(curString);
      curStream >> dummy >> dummy >> id;
      if (elem.id == id) {
        elementFound = true;
        curStream.seekg(0, curStream.beg);
        curStream >> tempPose;
        poseMap.emplace(id, std::move(tempPose));
        break;
      }
    }

    if (!genPoseListFile.good()) {
      genPoseListFile.close();
      outFileNum++;
      if (outFileNum < usableThreads) {
        if (!elementFound) {
          idx--; // go back one index
        }
        std::cout << "opening " << outFileNum << std::endl;
        genPoseListFile.open((inFileName + "_GeneratedPoses_" +
                              std::to_string(outFileNum) + ".txt"),
                             std::ios::in);
      } else {
        if (idx + 1 < sortedPoseCostVec.size()) {
          std::cout << "No more files to read " << idx << " "
                    << sortedPoseCostVec.size() << std::endl;
        }
        break;
      }
    } else {
      if (!elementFound) {
        std::cerr << "element not found" << elem.id << std::endl;
      }
    }
  }
  // close the file
  genPoseListFile.close();

  if (poseMap.size() != sortedPoseCostVec.size()) {
    std::cerr << "PoseMap and PoseCostVec doesnt match in size"
              << poseMap.size() << " " << sortedPoseCostVec.size() << std::endl;
    return false;
  }
  return true;
}

/**
 * @brief sortAndTrimPoseCosts
 * @param poseMap
 * @param sortedPoseCostVec
 * @param hYPbounds
 * @param torsoPosBound
 * @param torsoRotBound
 * @return
 */
bool sortAndTrimPoseCosts(PoseMap &poseMap,
                          std::vector<PoseCost> &sortedPoseCostVec,
                          const HeadYawPitch &hYPbounds,
                          const float &torsoPosBound,
                          const float &torsoRotBound) {
  /*
   * Sort the pose costs..
   * 1. Interaction count
   * 2. joint cost -> sum of interaction vector.
   */
  std::cout << "Sort pose costs" << std::endl;

  //  std::sort(sortedPoseCostVec.begin(), sortedPoseCostVec.end(),
  //            [](const PoseCost &i, const PoseCost &j) {
  //              // count is smaller, if same, then get lower cost.
  //              return i.interactionCount < j.interactionCount ||
  //                     (i.interactionCount == j.interactionCount &&
  //                      i.jointCost < j.jointCost);
  //            });

  ptrdiff_t grainSize = sortedPoseCostVec.size() / MAX_THREADS;
  utils::parallel_sort(sortedPoseCostVec.begin(), sortedPoseCostVec.size(),
                       grainSize, [](const PoseCost &i, const PoseCost &j) {
                         // count is smaller, if same, then get lower cost.
                         return i.interactionCount < j.interactionCount ||
                                (i.interactionCount == j.interactionCount &&
                                 i.jointCost < j.jointCost);
                       });
  /*
   * Trim poseVec based on closest-neighbour-like clustering.
   */
  NaoPose<float> curTopPose = poseMap.at(sortedPoseCostVec[0].id).pose;

  std::remove_if(
      sortedPoseCostVec.begin(), sortedPoseCostVec.end(),
      [&curTopPose, &hYPbounds, &torsoPosBound, &torsoRotBound,
       &poseMap](PoseCost &poseCost) {

        const NaoPose<float> &pose = poseMap.at(poseCost.id).pose;
        // remove poses that are near to the given bounds
        if (curTopPose.isNear(hYPbounds, torsoPosBound, torsoRotBound, pose)) {
          poseMap.erase(poseCost.id);
          return true;
        } else { // if out of the range, update this as curTopPose..
          curTopPose = pose;
          return false;
        }

      });

  return true;
}

/**
 * @brief populatePoseInteractions
 * @param poseInteractionList
 * @param sortedPoseCostVec
 * @return
 */
size_t
populatePoseInteractions(std::vector<PoseInteraction> &poseInteractionList,
                         const std::vector<PoseCost> &sortedPoseCostVec) {

  size_t approxMaxPosesToTry = std::min(
      sortedPoseCostVec.size(),
      static_cast<size_t>(std::round(std::sqrt(maxPoseInteractionCount * 2))));

  std::cout << approxMaxPosesToTry << std::endl;

  poseInteractionList.reserve(utils::getTriangleNum(approxMaxPosesToTry));

  for (size_t i = 0; i < approxMaxPosesToTry - 1; ++i) {
    auto &poseCostI = sortedPoseCostVec[i];
    for (size_t j = i + 1; j < approxMaxPosesToTry; ++j) {
      auto currentInteraction =
          PoseInteraction(poseCostI, sortedPoseCostVec[j]);
      // update the interaction cost for a pose.
      //      poseCostI.poseInteractionCost += currentInteraction.cost;
      poseInteractionList.emplace_back(std::move(currentInteraction));
    }
  }
  return approxMaxPosesToTry;
}

/**
 * @brief finalPoseFilteringAndChaining
 * Sort the pose interactions..
 * 1st priority, lowest interaction joint count
 * 2nd priority, lowest interaction cost
 * TODO 3rd priotity, lowest grand total of interaction costs for that joint.
 * @param poseInteractionList
 */
void poseToPoseInteractionSort(
    std::vector<PoseInteraction> &poseInteractionList) {
  /*
   *
   */
  std::cout << "Sort poseInteractions" << std::endl;
  std::sort(poseInteractionList.begin(), poseInteractionList.end(),
            [](const PoseInteraction &i, const PoseInteraction &j) -> bool {
              // count is smaller, if same, then get lower cost.
              return i.interactionCount < j.interactionCount ||
                     (i.interactionCount == j.interactionCount &&
                      i.cost < j.cost);
            });
  //  ptrdiff_t grainSize = poseInteractionList.size() / MAX_THREADS;

  //  utils::parallel_sort(
  //      poseInteractionList.begin(), poseInteractionList.size(), grainSize,
  //      [](const PoseInteraction &i, const PoseInteraction &j) -> bool {
  //        // count is smaller, if same, then get lower cost.
  //        return i.interactionCount < j.interactionCount ||
  //               (i.interactionCount == j.interactionCount && i.cost <
  //               j.cost);
  //      });
}

/**
 * @brief poseToPoseChainingMultiplication
 * @param idSet
 * @param poseInteractionList
 * @param levels
 * @return
 */
bool poseToPoseChaining(const PoseInteraction &rootInteraction,
                        std::vector<size_t> &idSet,
                        //    std::vector<PoseInteraction> &poseInteractionList,
                        //    const PoseMap &poseMap,
                        const std::vector<PoseCost> &poseCostVec,
                        const bool multiplicationMode = true,
                        size_t levels = 10) {

  std::vector<size_t> output;
  if (idSet.size() == 0 || poseCostVec.size() == 0) {
    return false;
  }

  auto finePoseCostById = [&poseCostVec](const size_t &id) {
    return std::find_if(poseCostVec.begin(), poseCostVec.end(),
                        [&id](const PoseCost &elem) { return elem.id == id; });

  };
  std::mutex idSetMtx;
  auto findCandidateForLevel = [&idSetMtx, &poseCostVec, &multiplicationMode,
                                &finePoseCostById](
      const std::vector<size_t>::iterator begin,
      const std::vector<size_t>::iterator end,
      const InteractionCost &curInteractionCostVec,
      unsigned int &globalTotalCount, unsigned int &globalTotalCost,
      size_t &globalBestCandidate) {

    unsigned int minTotalCount = 10E5;
    unsigned int minTotalCost = 10E5;
    size_t curBestCandidate = 0;

    for (auto candidateIdIter = begin; candidateIdIter != end;
         ++candidateIdIter) {
      size_t &candidateId = *candidateIdIter;
      if (candidateId == 0) {
        continue;
      }

      InteractionCost interactionCostVec;
      interactionCostVec.setZero();

      auto candidatePoseCost = finePoseCostById(candidateId);

      if (candidatePoseCost == poseCostVec.end()) {
        std::lock_guard<std::mutex> lg(utils::mtx_cout_);
        std::cerr << "Can't locate the pose" << std::endl;
        continue;
      }

      // get nonZeroCount
      unsigned int tempCost =
          static_cast<unsigned int>(std::abs(interactionCostVec.sum()));
      unsigned int tempCount = 0;

      for (Eigen::Index i = 0;
           i < static_cast<Eigen::Index>(TOT_AMBIGUITY_COMBOS); ++i) {
        if (multiplicationMode) {
          interactionCostVec(i) =
              candidatePoseCost->jointInteractionCostVec(i) *
              curInteractionCostVec(i);
        } else {
          interactionCostVec(i) =
              candidatePoseCost->jointInteractionCostVec(i) +
              curInteractionCostVec(i);
        }
        if (interactionCostVec(i) > 0) {
          tempCount++;
        }
      }

      // check if better than current best.
      if (tempCount < minTotalCount ||
          (tempCount == minTotalCount && tempCost < minTotalCost)) {
        minTotalCount = tempCount;
        minTotalCost = tempCost;
        curBestCandidate = candidateId;
        // remove this from the set
        {
          std::lock_guard<std::mutex> lg(idSetMtx);
          candidateId = 0;
        }
        //        idSet.erase(idSet.begin() + i);
      }
    }
    // compare the current best vs global best
    {
      std::lock_guard<std::mutex> lg(idSetMtx);
      // check if better than current best.
      if (minTotalCount < globalTotalCount ||
          (minTotalCount == globalTotalCount &&
           minTotalCost < globalTotalCost)) {
        globalTotalCount = minTotalCount;
        globalTotalCost = minTotalCost;
        globalBestCandidate = curBestCandidate;
      }
    }
  };

  //  auto first = poseInteractionList[0];
  if (rootInteraction.combinedId.first != idSet[0] ||
      rootInteraction.combinedId.second != idSet[1]) {
    std::cout << rootInteraction.combinedId.first << " "
              << rootInteraction.combinedId.second << "\n"
              << idSet[0] << " " << idSet[1] << "\n"
              << idSet[idSet.size() - 1] << " " << idSet[idSet.size() - 2]
              << std::endl;
  }
  std::cout << "Level 1 " << idSet[0] << "\nLevel 2 " << idSet[1] << std::endl;

  //  idSet.erase(idSet.begin(), idSet.begin() + 2); // remove top two
  output.emplace_back(idSet[0]);
  output.emplace_back(idSet[1]);
  idSet[0] = 0;
  idSet[1] = 0;

  size_t threadingOffsets = (idSet.size() - 2) / MAX_THREADS;

  // record the interaction cost of the initial chain
  InteractionCost curInteractionCostVec;
  {
    curInteractionCostVec.setZero();
    auto poseCost1 = finePoseCostById(rootInteraction.combinedId.first);
    auto poseCost2 = finePoseCostById(rootInteraction.combinedId.second);
    if (poseCost1 != poseCostVec.end() && poseCost2 != poseCostVec.end()) {
      int count = 0;
      PoseInteraction::getInteractionVecAndCount(curInteractionCostVec, count,
                                                 *poseCost1, *poseCost2);
    } else {
      std::cerr << "Can't map interaction costs for root again" << std::endl;
      return false;
    }
  }

  for (size_t levelIdx = 2; levelIdx < levels; ++levelIdx) {
    unsigned int minTotalCount = 10E5;
    unsigned int minTotalCost = 10E5;
    size_t curBestCandidate = 0;

    // update interaction cost of current chain
    {
      auto &candidateId = output.back(); // get the latest element
      auto poseCost = finePoseCostById(candidateId);
      for (Eigen::Index i = 0;
           i < static_cast<Eigen::Index>(TOT_AMBIGUITY_COMBOS); ++i) {
        curInteractionCostVec(i) =
            poseCost->jointInteractionCostVec(i) * curInteractionCostVec(i);
      }
    }
    // Start the threads
    std::vector<std::thread> threadList(MAX_THREADS);

    for (size_t i = 0; i < MAX_THREADS; ++i) {

      std::vector<size_t>::iterator begin = idSet.begin() + 2;
      std::advance(begin, (threadingOffsets * i));
      std::vector<size_t>::iterator end = idSet.begin();
      if (i + 1 >= MAX_THREADS) {
        end = idSet.end();
      } else {
        std::advance(end, (threadingOffsets * (i + 1)));
      }

      //      std::cout << "starting thread: " << i << std::endl;
      // Initialize the thread. Maybe use futures later?
      threadList[i] =
          std::thread(findCandidateForLevel, begin, end,
                      std::ref(curInteractionCostVec), std::ref(minTotalCount),
                      std::ref(minTotalCost), std::ref(curBestCandidate));
    }
    for (auto &thread : threadList) {
      if (thread.joinable()) {
        thread.join();
      }
    }
    output.emplace_back(curBestCandidate);
    std::cout << "Level " << levelIdx << " " << curBestCandidate << " "
              << minTotalCount << " " << minTotalCost << std::endl;
  }
  idSet = output;
  return true;
}

int main(int argc, char **argv) {
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
  filterAndSortPosesThreaded(usableThreads, inputPoseAndJointStreams,
                             sensorMagnitudeWeights, jointWeights,
                             wOrthoGeneric, observableJointCountWeight,
                             sortedPoseCostVec);
  /*
     * Start mapping pose cost to poses
     */
  std::map<size_t, poseAndRawAngleT> poseMap;
  mapPoseCostToPoses(poseMap, inFileName, sortedPoseCostVec, usableThreads);

  /*
   * Sort the pose costs..
   */
  // TODO get this from config.
  HeadYawPitch hYPbounds(1, 1);
  float torsoPosBound = 2.8f;
  float torsoRotBound = 5.1f;

  // if fails, we have a problem
  if (!sortAndTrimPoseCosts(poseMap, sortedPoseCostVec, hYPbounds,
                            torsoPosBound, torsoRotBound)) {
    return 1;
  }
  /*
   * Secondary sort run with poseInteractionns
   */
  std::vector<PoseInteraction> poseInteractionList;
  size_t approxMaxPosesToTry =
      populatePoseInteractions(poseInteractionList, sortedPoseCostVec);
  poseToPoseInteractionSort(poseInteractionList);

  // populate Id vec
  PoseInteraction rootPoseInteraction =
      poseInteractionList[0]; // this is the best
  std::vector<size_t> poseIdVec;
  {
    std::unordered_set<size_t> poseIdSet; // this keeps the poses as ordered
    // before. But being a set, we ensure
    // only a unique set of elems are
    // here.
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
                            multiplicationMode, 20)) {
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
