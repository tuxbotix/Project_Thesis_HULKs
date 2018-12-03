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

using JointWeights = std::array<float, JOINTS::JOINT::JOINTS_MAX>;
using SensorWeights = std::array<float, SENSOR_COUNT>;
using SensMagAndPoseId =
    std::pair<int, size_t>; // Sensor Name should be maintained externally.

inline size_t getSmallerTriangleNum(const size_t &index);

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

struct PoseInteraction {
  double cost;
  PoseInteractionId combinedId; // id1, id2 in order
  // In this case, my approach is elementwise multiplication
//  InteractionCost
//      poseInteractionCostVec; // p1 vs p2, p1 vs p3, ... for all joint
                              // configs under calibration
  int interactionCount;       // count of non-zero elements of interaction cost
  PoseInteraction(PoseCost &c1, PoseCost &c2) {
    if (c1.id > c2.id) {
      combinedId = {c2.id, c1.id};
    } else {
      combinedId = {c1.id, c2.id};
    }
    //    interactionCost = (c1.interactionCost - c2.interactionCost).
    InteractionCost
        poseInteractionCostVec; // p1 vs p2, p1 vs p3, ... for all joint
    for (long i = 0; i < TOT_AMBIGUITY_COMBOS; ++i) {
      poseInteractionCostVec(i) = std::abs(c1.jointInteractionCostVec(i) *
                                           c2.jointInteractionCostVec(i));
      if (poseInteractionCostVec(i) > 0) {
        interactionCount++;
      }
    }
    cost = poseInteractionCostVec.sum();
  }
};

const size_t maxPoseInteractionCount =
    MAX_MEM_FOR_PIP / sizeof(PoseInteraction);

/**
 * @brief getSmallerTriangleNum Get the next (smaller) sized triangle num
 * @param sizeOfSize
 * @return
 */
inline size_t getSmallerTriangleNum(const size_t &sizeOfSize) {
  return sizeOfSize * (sizeOfSize - 1) / 2;
}

/**
 * @brief getTriangleNum
 * @param sizeOfSize
 * @return
 */
inline size_t getTriangleNum(const size_t &sizeOfSize) {
  return sizeOfSize * (sizeOfSize + 1) / 2;
}

/**
 * @brief getIndexUpperTriangle Get begin index to fill in the cost arr..
 * @return
 */
template <typename T = size_t>
inline T getIndexUpperTriangle(const size_t &i, const size_t &j) {
  return static_cast<T>(TOT_AMBIGUITY_COMBOS - getSmallerTriangleNum(i) + j -
                        2);
  //  return static_cast<T>((i * ((2 * TOT_OBS_JOINT_COUNT) - 1 - i) / 2) + j);
}

// static const size_t maxPeakElemsPerList = 10E15;
// static const size_t maxPeakElemsPerListSortTrigger = 10E18;
// static const size_t finalPoseFilteringLimit = 1000;

// void sortDescAndTrimPoseCostVector(std::vector<PoseCost> &vec,
//                                   const size_t &elemLimit) {
//  // sort in asc. order
//  std::sort(vec.begin(), vec.end());
//  // erase the excessive elements.
//  if (vec.size() > elemLimit) {
//    vec.erase(vec.begin() + elemLimit, vec.end());
//  }
//}

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
  double accum = observableJointCountWeight * p.getObservableCount();
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
    const float wMag = wSensorMag * wJoints[j];
    const float wOrtho = wOrthoGeneric * wJoints[j];
    T val;
    bool obs;
    p.getSensitivity(joint, val, obs);
    // stage 1
    accum += wMag * val.norm();
    if (std::isnan(val.norm()) || std::isnan(accum)) {
      std::lock_guard<std::mutex> lock(utils::mtx_cout_);
      std::cout << p.getId() << " is NAN 1st stage joint: " << j << " val "
                << val.norm() << std::endl;
      continue;
    }

    double v1Norm = val.norm();
    for (size_t k = j + 1; k < TOT_OBS_JOINT_COUNT; k++) {
      const JOINTS::JOINT innerJoint = static_cast<JOINTS::JOINT>(k);
      // skip own-joint and unobservable joints..
      if (!p.isJointObservable(innerJoint)) {
        continue;
      }
      T val2;
      bool obs2;
      p.getSensitivity(innerJoint, val2, obs2);

      double v2Norm = val2.norm();
      // second stage
      double dot = std::fabs((double)val.dot(val2) / (v1Norm * v2Norm));
      // We trust in the dot product :P
      if (dot > 1.0) {
        dot = 1.0;
      } else if (dot < -1.0) {
        dot = -1.0;
      }
      // get the linear angle value.. closer to 90deg, the better.

      double curCost = static_cast<double>(wOrtho) *
                       std::fabs(std::acos(dot) / static_cast<double>(TO_RAD));
      const long curIdx = getIndexUpperTriangle<Eigen::Index>(j, k);
      interactionCost(curIdx) = static_cast<int>(std::round(curCost));

      accum += curCost;
      if (dot > (double)1.0 || dot < (double)-1.0 || std::isnan(dot) ||
          std::isnan(accum)) {
        std::lock_guard<std::mutex> lock(utils::mtx_cout_);
        std::cout << p.getId() << " is NAN 2nd stage [domain err] " << j << " "
                  << k << " dot " << dot << " aco " << std::acos(dot)
                  << " wOrtho " << wOrtho << " fabs "
                  << std::fabs(std::acos(dot) / TO_RAD) << std::endl;
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
void poseFilterFunc(std::istream &inputStream,
                    // std::ostream &outputStream,
                    std::vector<PoseCost> &poseCosts,
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

  //  InteractionCost interactionCost;
  //  interactionCost.setZero();
  //  double poseCostAccum = 0;

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
      for (auto i = 0; i < TOT_AMBIGUITY_COMBOS; ++i) {
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
      //            if (poseCosts.size() > maxPeakElemsPerListSortTrigger)
      //            {
      //                sortDescAndTrimPoseCostVector(poseCosts,
      //                maxPeakElemsPerList);
      //            }

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

  // std::cout << "finishing" << std::endl;

  // sort ASC by cost and trim
  //    sortDescAndTrimPoseCostVector(poseCosts, maxPeakElemsPerList);
}

int main(int argc, char **argv) {
  std::string inFileName((argc > 1 ? argv[1] : "out"));
  std::string favouredJoint((argc > 2 ? argv[2] : "-1"));
  std::string favouredSensor((argc > 3 ? argv[3] : "all")); // t, b, all
  std::string confRoot((argc > 4 ? argv[4] : "../../nao/home/"));

  int jointNum = std::stoi(favouredJoint);
  if (jointNum < 0 || jointNum >= static_cast<int>(JOINTS::JOINT::JOINTS_MAX)) {
    jointNum = -1;
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
  {
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
    if (jointNum > 0 &&
        jointNum < static_cast<int>(JOINTS::JOINT::JOINTS_MAX)) {
      jointWeights[jointNum] = 4;
    }

    const float wOrthoGeneric = 3;
    const float observableJointCountWeight = 1;

    for (unsigned int i = 0; i < usableThreads; i++) {

      // void poseFilterFunc(std::istream &inputStream,
      //                     // std::ostream &outputStream,
      //                     std::vector<PoseCost> &poseCosts,
      //                     const SensorWeights sensorMagnitudeWeights,
      //                     const JointWeights jointWeights,
      //                     const float orthogonalityWeight,
      //                     std::atomic<size_t> &iterations, bool lastPortion)

      threadList[i] = std::thread(
          poseFilterFunc, std::ref(inputPoseAndJointStreams[i]),
          // std::ref(outputFileList[i]),
          std::ref(poseCostListList[i]), sensorMagnitudeWeights, jointWeights,
          wOrthoGeneric, observableJointCountWeight, std::ref(iterCount[i]));
    }
#if ENABLE_PROGRESS
    bool continueTicker = true;
    std::thread tTick = std::thread([&]() {
      int elapsed = 0;
      const size_t interval = 5;
      while (continueTicker) {
        size_t iterSum =
            std::accumulate(iterCount.begin(), iterCount.end(), (size_t)0);
        if (elapsed % 100) {
          std::lock_guard<std::mutex> lock(utils::mtx_cout_);
          std::cout << "Elapsed: " << elapsed
                    << "s Iterations: " << std::scientific << (double)iterSum
                    << std::endl;
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
    /// Join all :P
    for (auto &t : threadList) {
      if (t.joinable()) {
        t.join();
      }
    }
#if ENABLE_PROGRESS
    continueTicker = false;
    if (tTick.joinable()) {
      tTick.join();
    }
#endif

    /// Write the remaining buffers to file
    std::cout << "flushing all " << std::endl;
#if DO_COMMIT

    // first join all
    std::cout << poseCostListList[0].size() << " ";
    for (size_t t = 1; t < usableThreads; t++) // per thread
    {
      std::cout << poseCostListList[t].size() << " ";
      poseCostListList[0].insert(
          poseCostListList[0].end(),
          std::make_move_iterator(poseCostListList[t].begin()),
          std::make_move_iterator(poseCostListList[t].end()));
    }
    std::cout << std::endl;
    std::vector<PoseCost> &sortedPoseCostVec = poseCostListList[0];

    std::cout << "total filtered pose count " << sortedPoseCostVec.size()
              << std::endl;
    std::map<size_t, poseAndRawAngleT> poseMap;
    std::map<size_t, poseAndRawAngleT>::iterator poseMapIter;

    std::fstream genPoseListFile;

    size_t outFileNum = 0;
    std::cout << "opening " << outFileNum << std::endl;
    genPoseListFile.open(
        (inFileName + "_GeneratedPoses_" + std::to_string(outFileNum) + ".txt"),
        std::ios::in);

    for (size_t idx = 0; idx < sortedPoseCostVec.size(); idx++) // per thread
    {
      auto &elem = sortedPoseCostVec[idx];

      std::string curString = "";
      std::string dummy = "";
      size_t id = 0;
      bool elementFound = false;

      poseAndRawAngleT tempPose;

      while (std::getline(genPoseListFile, curString)) {
        //                triedLines ++;

        std::stringstream curStream(curString);

        curStream >> dummy >> dummy >> id;
        if (elem.id == id) {
          elementFound = true;
          curStream.seekg(0, curStream.beg);

          curStream >> tempPose;
          poseMap.emplace(id, std::move(tempPose));
          //                    curStream >> poseList[idx];
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
          std::cout << "element not found" << elem.id << std::endl;
        }
      }
    }
    // close the file
    genPoseListFile.close();

    /*
     * Sort the pose costs..
     */
    std::cout << "Sort pose costs" << std::endl;
    std::sort(sortedPoseCostVec.begin(), sortedPoseCostVec.end(),
              [](PoseCost &i, PoseCost &j) {
                // count is smaller, if same, then get lower cost.
                return i.interactionCount < j.interactionCount ||
                       (i.interactionCount == j.interactionCount &&
                        i.jointCost < j.jointCost);
              });

    if (poseMap.size() != sortedPoseCostVec.size()) {
      std::cerr << "PoseMap and PoseCostVec doesnt match in size"
                << poseMap.size() << " " << sortedPoseCostVec.size()
                << std::endl;
      return 1;
    }
    {
      /*
       * Trim poseVec based on closest-neighbour-like clustering.
       */
      NaoPose<float> &curTopPose = poseMap.at(sortedPoseCostVec[0].id).pose;
      HeadYawPitch hYPbounds(1, 1);
      float torsoPosBound = 2.8f;
      float torsoRotBound = 5.1f;

      std::remove_if(
          sortedPoseCostVec.begin(), sortedPoseCostVec.end(),
          [&curTopPose, &hYPbounds, &torsoPosBound, &torsoRotBound,
           &poseMap](PoseCost &poseCost) {

            NaoPose<float> &pose = poseMap.at(poseCost.id).pose;
            // remove poses that are near to the given bounds
            if (curTopPose.isNear(hYPbounds, torsoPosBound, torsoRotBound,
                                  pose)) {
              return true;
            } else { // if out of the range, update this as curTopPose..
              curTopPose = pose;
              return false;
            }

          });
    }
    /*
     * Secondary sort run with poseInteractionns
     */

    size_t approxMaxPosesToTry =
        std::min(sortedPoseCostVec.size(),
                 static_cast<size_t>(
                     std::round(std::sqrt(maxPoseInteractionCount * 2))));

    std::cout << approxMaxPosesToTry << std::endl;

    std::vector<PoseInteraction> poseInteractionList;
    poseInteractionList.reserve(getTriangleNum(approxMaxPosesToTry));

    for (size_t i = 0; i < approxMaxPosesToTry; ++i) {
      auto &poseCostI = sortedPoseCostVec[i];
      for (size_t j = i + 1; j < approxMaxPosesToTry; ++j) {
        auto currentInteraction =
            PoseInteraction(poseCostI, sortedPoseCostVec[j]);
        // update the interaction cost for a pose.
        poseCostI.interactionCount += currentInteraction.cost;
        poseInteractionList.emplace_back(std::move(currentInteraction));
      }
    }
    /*
     * Sort the pose interactions..
     * 1st priority, lowest interaction joint count
     * 2nd priority, lowest interaction cost
     * 3rd priotity, lowest grand total of interaction costs for that joint.
     * TODO implement the last..
     */
    std::cout << "Sort poseInteractions" << std::endl;
    std::sort(poseInteractionList.begin(), poseInteractionList.end(),
              [](PoseInteraction &i, PoseInteraction &j) {
                // count is smaller, if same, then get lower cost.
                return i.interactionCount < j.interactionCount ||
                       (i.interactionCount == j.interactionCount &&
                        i.cost < j.cost);
              });

    /*
     * Now write to file..
     */
    std::unordered_set<size_t> poseIdSet; // this keeps the poses as ordered
                                          // before. But being a set, we ensure
                                          // only a unique set of elems are
                                          // here.
    for (auto &elem : poseInteractionList) {
      poseIdSet.insert(elem.combinedId.first);
      poseIdSet.insert(elem.combinedId.second);
    }
    std::cout << "Commit to file.." << poseIdSet.size() << " "
              << poseInteractionList.size() << std::endl;
    size_t iter = 0;
    for (auto &id : poseIdSet) {
      auto pose = poseMap.at(id);
      try {
        auto result =
            std::find_if(sortedPoseCostVec.begin(), sortedPoseCostVec.end(),
                         [&id](PoseCost &c) { return c.id == id; });
        if (result != sortedPoseCostVec.end()) {
          //                        poseList[i].pose.
          outFilteredPosesFile << pose << std::endl;
          outCostsFile << "poseCost " << result->id << " " << result->jointCost
                       << std::endl;
          iter++;
        }
      } catch (std::exception e) {
        std::cerr << e.what() << std::endl;
        return 1;
      }
    }

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
// poseFilterConf.json
