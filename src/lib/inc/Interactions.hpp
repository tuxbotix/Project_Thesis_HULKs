#pragma once

#include <cmath>
#include <cstddef>
//#include <gmpxx.h>

#include "JointCalibSolver.hpp"
#include "NaoPoseInfo.hpp"
#include "utils.hpp"

#ifndef OVERFLOW_DETECT
#define OVERFLOW_DETECT 1
#endif

namespace Interactions {

/* Constants */
const size_t TOT_OBS_JOINT_COUNT =
    JointCalibSolvers::COMPACT_JOINT_CALIB_PARAM_COUNT;

const size_t TOT_AMBIGUITY_COMBOS =
    TOT_OBS_JOINT_COUNT * (TOT_OBS_JOINT_COUNT - 1) / 2;

const double TOL = 0.999;

/* Aliases */
using PoseMap = std::map<size_t, poseAndRawAngleT>;
using InteractionCostType = int64_t;
using InteractionCost =
    Eigen::Array<InteractionCostType, TOT_AMBIGUITY_COMBOS, 1>;
using PoseInteractionId = std::pair<size_t, size_t>; // id1, id2 in order

const size_t PartialMeanTriggerLevel = 5; // trigger every 5th level

/**
 * @brief The PoseCost struct
 */
struct PoseCost {
  enum DataType { NONE, UINT32, INT32, UINT64, INT64, FLOAT, DOUBLE };

  static DataType getTypeEnum() {
    if (typeid(InteractionCostType) == typeid(double)) {
      return DataType::DOUBLE;
    } else if (typeid(InteractionCostType) == typeid(uint32_t)) {
      return DataType::UINT32;
    } else if (typeid(InteractionCostType) == typeid(int32_t)) {
      return DataType::INT32;
    } else if (typeid(InteractionCostType) == typeid(uint64_t)) {
      return DataType::UINT64;
    } else if (typeid(InteractionCostType) == typeid(int64_t)) {
      return DataType::INT64;
    } else if (typeid(InteractionCostType) == typeid(float)) {
      return DataType::FLOAT;
    } else {
      return DataType::NONE;
    }
  }

  size_t id;
  double jointCost;
  InteractionCost
      jointInteractionCostVec; // j1 vs j2, j1 vs j3, ... for all joint
                               // configs under calibration
  int interactionCount;        // count of non-zero elements of interaction cost

  PoseCost(const double &cost, const size_t &id)
      : id(id), jointCost(cost), interactionCount(0) {
    jointInteractionCostVec.setZero();
  }
  void reset() {
    jointCost = 0;
    id = 0;
    jointInteractionCostVec.setZero();
    interactionCount = 0;
  }

  std::ostream &write(std::ostream &out) {
    char dataType = static_cast<char>(getTypeEnum());
    out.write(&dataType, sizeof(dataType));
    out.write(reinterpret_cast<const char *>(&id), sizeof(id));
    out.write(reinterpret_cast<const char *>(&jointCost), sizeof(jointCost));
    for (Eigen::Index i = 0; i < jointInteractionCostVec.size(); ++i) {
      out.write(reinterpret_cast<const char *>(&jointInteractionCostVec(i)),
                sizeof(jointInteractionCostVec(i)));
    }
    out.write(reinterpret_cast<const char *>(&interactionCount),
              sizeof(interactionCount));
    return out;
  }

  std::istream &read(std::istream &in) {
    char dataType = 0;
    in.read(&dataType, sizeof(dataType));
    if (dataType != static_cast<char>(getTypeEnum())) {
      std::cerr << "Non matching type for pose cost!!!" << std::endl;
      throw("Non matching type for pose cost!!!");
    }
    in.read(reinterpret_cast<char *>(&id), sizeof(id));
    in.read(reinterpret_cast<char *>(&jointCost), sizeof(jointCost));
    for (Eigen::Index i = 0; i < jointInteractionCostVec.size(); ++i) {
      in.read(reinterpret_cast<char *>(&jointInteractionCostVec(i)),
              sizeof(jointInteractionCostVec(i)));
    }
    in.read(reinterpret_cast<char *>(&interactionCount),
            sizeof(interactionCount));
    return in;
  }

  void updateInteractionCount() {
#if OVERFLOW_DETECT
    if ((jointInteractionCostVec < 0.0).any()) {
      std::cout << "OVERFLOW @ updateInteractionCount" << std::endl;
    }
#endif
    interactionCount = static_cast<int>(
        (jointInteractionCostVec > static_cast<InteractionCostType>(TOL))
            .count());
    //    for (Eigen::Index i = 0; i < jointInteractionCostVec.size(); ++i) {
    //      // 0.99 -> 0 for int and so on. I just need values greater or equal
    //      to 1.0
    //      if (std::abs(jointInteractionCostVec(i)) >
    //          static_cast<InteractionCostType>(TOL)) {
    //        interactionCount++;
    //      }
    //#if OVERFLOW_DETECT
    //      if (jointInteractionCostVec(i) < 0.0) {
    //        std::cout << "OVERFLOW @ updateInteractionCount" << std::endl;
    //      }
    //#endif
    //    }
  }
};

/**
 * @brief makeCombinedId
 * @param id1
 * @param id2
 * @return
 */
inline PoseInteractionId makeCombinedId(const size_t &id1, const size_t &id2) {
  return (id1 < id2) ? PoseInteractionId{id1, id2}
                     : PoseInteractionId{id2, id1};
}

/**
 * @brief The PoseInteraction struct
 */
struct PoseInteraction {
  PoseInteractionId combinedId; // id1, id2 in order
  double cost;
  int interactionCount;
  PoseInteraction(const PoseCost &c1, const PoseCost &c2)
      : combinedId(makeCombinedId(c1.id, c2.id)), cost(0), interactionCount(0) {

    //    interactionCost = (c1.interactionCost - c2.interactionCost).
    InteractionCost
        poseInteractionCostVec; // p1 vs p2, p1 vs p3, ... for all joint
    poseInteractionCostVec.setZero();

    cost = PoseInteraction::getInteractionVecAndCount(
        poseInteractionCostVec, c1.jointInteractionCostVec,
        c2.jointInteractionCostVec);
  }

  /**
   * @brief getInteractionVecAndCount
   * @param interactionCostVec
   * @param c1
   * @param c2
   * @return
   */
  inline static int
  getInteractionVecAndCount(InteractionCost &interactionCostVec,
                            const InteractionCost &c1,
                            const InteractionCost &c2) {
    int interactionCount = 0;

    interactionCostVec = c1 * c2;
    interactionCostVec.sqrt(); // geometric average between the two poses

// TODO check if .any() is better than this.
#if OVERFLOW_DETECT
    for (Eigen::Index i = 0;
         i < static_cast<Eigen::Index>(TOT_AMBIGUITY_COMBOS); ++i) {
      if (interactionCostVec(i) < 0.0 && c1(i) > 0.0 && c2(i) > 0.0) {
        std::cout << "OVERFLOW @ getInteractionVecAndCount" << std::endl;
      }
    }
#endif
    interactionCount = static_cast<int>(
        (interactionCostVec > static_cast<InteractionCostType>(TOL)).count());
    //    for (Eigen::Index i = 0;
    //         i < static_cast<Eigen::Index>(TOT_AMBIGUITY_COMBOS); ++i) {
    //      //      interactionCostVec(i) = c1(i) * c2(i);
    //      if (std::abs(interactionCostVec(i)) >
    //          static_cast<InteractionCostType>(TOL)) {
    //        interactionCount++;
    //      }
    //#if OVERFLOW_DETECT
    //      if (interactionCostVec(i) < 0.0 && c1(i) > 0.0 && c2(i) > 0.0) {
    //        std::cout << "OVERFLOW @ getInteractionVecAndCount" << std::endl;
    //      }
    //#endif
    //    }
    return interactionCount;
  }

  /**
   * @brief getInteractionVecAndCount Getting the geometric average is user's
   * responsibilioty
   * @param interactionCostVec
   * @param interactionCount
   * @param c1
   * @param c2
   */
  inline static void
  getInteractionVecAndCount(InteractionCost &interactionCostVec,
                            int &interactionCount, const PoseCost &c1,
                            const PoseCost &c2) {
    interactionCount = getInteractionVecAndCount(interactionCostVec,
                                                 c1.jointInteractionCostVec,
                                                 c2.jointInteractionCostVec);
  }
};

/**
 * @brief encodeInteractionVecIndex Get begin index to fill in the cost arr..
 * Constraints:
 *  1. min(row) = 1, min(col) = 0.
 *  2. row != col
 *  3. row > col
 *
 * Layout can be shown graphically as follows;
 *  row and column beginning is already satisfied
 *  numbers in bottom right cell are valid indexes
 *  e is case 2., b is case 3.
 *
 *   | 0 1 2 3 4 5
 * --+ ------------
 * 1 | 0 e b b b b
 * 2 | 1 2 e b b b
 * 3 | 3 4 5 e b b
 * 4 | 6 7 8 9 e b
 *
 * @return index (0 index)
 */
template <typename T = size_t>
inline T encodeInteractionVecIndex(const size_t &row, const size_t &col) {
  if (row == col || row <= 0 || col >= row) {
    std::cerr << "Invalid indexes, j > 0 or i >= j  violated " << row << " "
              << col << std::endl;
    throw("Triangle index probvlem");
  }
  return static_cast<T>(utils::getSmallerTriangleNum(row) + col);
}

template <typename T = size_t>
/**
 * @brief decodeInteractionVecIndex. Refer to encoder function for details
 * @param index
 * @param i
 * @param j
 * @return
 */
inline void decodeInteractionVecIndex(const T &index, size_t &row,
                                      size_t &col) {
  for (size_t iter = 1; iter < utils::TRIANGLE_NUMS_30.size(); ++iter) {
    if (index < static_cast<T>(utils::TRIANGLE_NUMS_30[iter])) {
      col = static_cast<size_t>(index) - utils::TRIANGLE_NUMS_30[iter - 1];
      row = iter;
      break;
    }
  }
  if (encodeInteractionVecIndex(row, col) != static_cast<size_t>(index)) {
    std::cerr << "Reconstruction failure. " << index << " " << row << " " << col
              << std::endl;
    throw("Triangle Reconstruction failure");
  }
}

template <typename T = size_t>
inline std::pair<size_t, size_t> decodeInteractionVecIndex(const T &index) {
  size_t i, j;
  decodeInteractionVecIndex(index, i, j);
  return {i, j};
}
/*
 * Pose mapping and stuff
 */

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
                          const Vector3f &torsoPosBound,
                          const Vector3f &torsoRotBound,
                          const size_t MAX_THREADS = 4) {
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

  size_t grainSize = sortedPoseCostVec.size() / MAX_THREADS;
  utils::parallel_sort(sortedPoseCostVec.begin(), sortedPoseCostVec.size(),
                       grainSize, [](const PoseCost &i, const PoseCost &j) {
                         // count is smaller, if same, then get lower cost.
                         return i.jointCost < j.jointCost;
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

/*
 * Interaction chaining and stuff
 */

/**
 * @brief populatePoseInteractions
 * @param poseInteractionList
 * @param sortedPoseCostVec
 * @return
 */
size_t
populatePoseInteractions(std::vector<PoseInteraction> &poseInteractionList,
                         const std::vector<PoseCost> &sortedPoseCostVec,
                         const size_t maxPoseInteractionCount) {

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

inline bool poseInteractionCriteria(const PoseInteraction &i,
                                    const PoseInteraction &j) {
  // count is smaller, if same, then get lower cost.
  return i.interactionCount < j.interactionCount ||
         (i.interactionCount == j.interactionCount && i.cost < j.cost);
};

inline bool poseInteractionCriteria(const int &iCount, const double &iCost,
                                    const int &jCount, const double &jCost) {
  // count is smaller, if same, then get lower cost.
  return (iCount < jCount || (iCount == jCount && iCost < jCost));
};
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
                        size_t levels = 10, const size_t MAX_THREADS = 4) {

  std::vector<size_t> output;
  if (idSet.size() == 0 || poseCostVec.size() == 0) {
    return false;
  }

  auto findPoseCostById = [&poseCostVec](const size_t &id) {
    return std::find_if(poseCostVec.begin(), poseCostVec.end(),
                        [&id](const PoseCost &elem) { return elem.id == id; });

  };
  std::mutex idSetMtx;
  /*
   * Curlevel is 1 index
   */
  auto findCandidateForLevel = [&idSetMtx, &poseCostVec, &findPoseCostById](
      const std::vector<size_t>::iterator begin,
      const std::vector<size_t>::iterator end,
      const InteractionCost curInteractionCostVec, int &globalTotalCount,
      double &globalTotalCost, size_t &globalBestCandidate,
      size_t curLevel1Index) {

    int minTotalCount = 1E3;
    double minTotalCost = 1E6;
    size_t curBestCandidate = 0;

    for (auto candidateIdIter = begin; candidateIdIter != end;
         ++candidateIdIter) {
      size_t &candidateId = *candidateIdIter;
      if (candidateId == 0) {
        continue;
      }

      InteractionCost interactionCostVec;
      interactionCostVec.setZero();

      auto candidatePoseCost = findPoseCostById(candidateId);

      if (candidatePoseCost == poseCostVec.end()) {
        std::lock_guard<std::mutex> lg(utils::mtx_cout_);
        std::cerr << "Can't locate the pose" << std::endl;
        continue;
      }

      // get nonZeroCount
      int tempCount = 0;
      tempCount = PoseInteraction::getInteractionVecAndCount(
          interactionCostVec, candidatePoseCost->jointInteractionCostVec,
          curInteractionCostVec);

      //      double tempCost = (interactionCostVec.cast<double>()
      //                             .abs()
      //                             .pow(1.0 / curLevel1Index)
      //                             .sum() /
      //                         static_cast<double>(TOT_AMBIGUITY_COMBOS));
      double tempCost = pow((interactionCostVec.cast<double>().abs().sum() /
                             TOT_AMBIGUITY_COMBOS),
                            1.0 / curLevel1Index);

      // get geometric mean~
      //      tempCost = pow(tempCost / TOT_AMBIGUITY_COMBOS, 1 /
      //      curLevel1Index);
      // check if better than current best.
      if (poseInteractionCriteria(tempCount, tempCost, minTotalCount,
                                  minTotalCost)) {
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

      if (poseInteractionCriteria(minTotalCount, minTotalCost, globalTotalCount,
                                  globalTotalCost)) {
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

  // start chaining.
  output.emplace_back(idSet[0]);
  output.emplace_back(idSet[1]);
  idSet[0] = 0;
  idSet[1] = 0;

  size_t threadingOffsets = (idSet.size() - 2) / MAX_THREADS;

  int curMinCostCount = 0;
  double curMinTotalCost = 0;
  // record the interaction cost of the initial chain
  InteractionCost curInteractionCostVec;
  {
    curInteractionCostVec.setZero();
    auto poseCost1 = findPoseCostById(rootInteraction.combinedId.first);
    auto poseCost2 = findPoseCostById(rootInteraction.combinedId.second);
    if (poseCost1 != poseCostVec.end() && poseCost2 != poseCostVec.end()) {
      PoseInteraction::getInteractionVecAndCount(
          curInteractionCostVec, curMinCostCount, *poseCost1, *poseCost2);
    } else {
      std::cerr << "Can't map interaction costs for root again" << std::endl;
      return false;
    }
    curMinTotalCost =
        std::sqrt(curInteractionCostVec.cast<double>().abs().sum() /
                  static_cast<double>(TOT_AMBIGUITY_COMBOS));

    std::cout << "Level 1 " << output[0]
              << " poseCostRank: " << (poseCost1 - poseCostVec.begin()) << "\t"
              << poseCost1->interactionCount << " "
              << poseCost1->jointInteractionCostVec.cast<double>().abs().sum() /
                     static_cast<double>(TOT_AMBIGUITY_COMBOS)
              << "\nLevel 2 " << output[1]
              << " poseCostRank: " << (poseCost2 - poseCostVec.begin()) << "\t"
              << curMinCostCount << " " << curMinTotalCost << std::endl;
  }

  for (size_t levelIdx = 2; levelIdx < levels; ++levelIdx) {
    int minTotalCount = 10E5;
    double minTotalCost = 10E5;
    size_t curBestCandidate = 0;

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
          std::thread(findCandidateForLevel, begin, end, curInteractionCostVec,
                      std::ref(minTotalCount), std::ref(minTotalCost),
                      std::ref(curBestCandidate), (levelIdx + 1));
    }
    for (auto &thread : threadList) {
      if (thread.joinable()) {
        thread.join();
      }
    }

    if (curMinCostCount < minTotalCount ||
        (curMinCostCount == minTotalCount && curMinTotalCost < minTotalCost)) {
      std::cout
          << "Attempted level " << levelIdx + 1
          << " No convergence; exiting chaining. cCount, cost, mCount, cost"
          << curMinCostCount << " " << curMinTotalCost << " " << minTotalCount
          << " " << minTotalCost << std::endl;
      break;
    } else {
      curMinCostCount = minTotalCount;
      curMinTotalCost = minTotalCost;
    }
    // update interaction cost of current chain
    {
      auto poseCost = findPoseCostById(curBestCandidate);
      // current chain's cost count
      //      if (levelIdx % PartialMeanTriggerLevel != 0) {
      int laa = PoseInteraction::getInteractionVecAndCount(
          curInteractionCostVec, poseCost->jointInteractionCostVec,
          curInteractionCostVec);
      if (laa != curMinCostCount) {
        std::cout << "We got a problem\n";
      }
      //      } else {
      //      }

      // print current level, best candidate, totalCount of the chain, best
      // count
      // for this level, best cost for this stage.

      std::cout << "Level " << levelIdx + 1 << " " << curBestCandidate
                << " poseCostRank: " << (poseCost - poseCostVec.begin()) << "\t"
                << curMinCostCount << " " << minTotalCost << std::endl;
    }
    output.emplace_back(curBestCandidate);
  }
  idSet = output;
  return true;
}

} // namespace Interactions
