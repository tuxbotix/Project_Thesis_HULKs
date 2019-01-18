// #include <algorithm>
// #include <assert.h>
// #include <cmath>
// #include <fstream>
// #include <iostream>
// #include <limits>
// #include <numeric>
// #include <thread>

// #include <Data/CameraMatrix.hpp>
// #include <Modules/Configuration/Configuration.h>
// #include <Modules/Configuration/UnixSocketConfig.hpp>
// #include <Modules/NaoProvider.h>

// #include <Hardware/RobotInterface.hpp>

// #include <cxxopts/include/cxxopts.hpp>

// #include "Interactions.hpp"
// #include "MiniConfigHandle.hpp"
// #include "NaoPoseInfo.hpp"
// #include "ObservationSensitivityProvider.hpp"
// #include "TUHHMin.hpp"
// #include "constants.hpp"

// #define DO_COMMIT 1
// #define WRITE_PARALLEL 1
// #define DEBUG_FILE_COMMIT 1
// #define DEBUG_IN_THREAD 1
// #define ENABLE_PROGRESS 1

// #include "utils.hpp"

// static const size_t MAX_MEM_FOR_PIP =
//    20E9; // max mmory for pose vs pose interactions

// const unsigned int MAX_THREADS = std::thread::hardware_concurrency();

// /**
// *
// */
// using namespace Interactions;

// const size_t maxPoseInteractionCount =
//    MAX_MEM_FOR_PIP / sizeof(PoseInteraction);

// PoseInteraction getBestPoseInteraction(const std::vector<PoseCost> &vec,
//                                       size_t maxThreads = 1) {
//  const size_t approxMaxPosesToTry = vec.size();

//  std::cout << "Start getBestInteraction " << approxMaxPosesToTry << std::endl;

//  std::mutex mtx;
//  auto fcn = [&mtx](const std::vector<PoseCost>::const_iterator begin,
//                    const std::vector<PoseCost>::const_iterator end,
//                    const std::vector<PoseCost>::const_iterator globalEnd,
//                    PoseInteraction &globalBest) {

//    PoseInteraction curBest(PoseCost(std::numeric_limits<double>::max(), 0),
//                            PoseCost(std::numeric_limits<double>::max(), 0));

//    for (auto i = begin; i != end - 1; ++i) {
//      auto &poseCostI = *i;
//      for (auto j = i + 1; j != globalEnd; ++j) {
//        auto &poseCostJ = *j;
//        auto currentInteraction = PoseInteraction(poseCostI, poseCostJ);
//        if (currentInteraction.interactionCount < curBest.interactionCount ||
//            (currentInteraction.interactionCount == curBest.interactionCount &&
//             currentInteraction.cost < curBest.cost)) {
//          curBest = currentInteraction;
//        }
//      }
//    }
//    {
//      std::lock_guard<std::mutex> lg(mtx);
//      if (curBest.interactionCount < globalBest.interactionCount ||
//          (curBest.interactionCount == globalBest.interactionCount &&
//           curBest.cost < globalBest.cost)) {
//        globalBest = curBest;
//      }
//    }
//  };

//  std::vector<std::thread> threadList(maxThreads);
//  const size_t threadingOffsets = (vec.size()) / maxThreads;

//  PoseInteraction bestInteraction(
//      PoseCost(std::numeric_limits<double>::max(), 0),
//      PoseCost(std::numeric_limits<double>::max(), 0));

//  for (size_t i = 0; i < maxThreads; ++i) {

//    std::vector<PoseCost>::const_iterator begin = vec.begin();
//    std::advance(begin, (threadingOffsets * i));
//    std::vector<PoseCost>::const_iterator end = vec.begin();
//    if (i + 1 >= maxThreads) {
//      end = vec.end();
//    } else {
//      std::advance(end, (threadingOffsets * (i + 1)));
//    }
//    threadList[i] = std::thread(fcn, begin, end, std::ref(bestInteraction));
//  }
//  for (auto &thread : threadList) {
//    if (thread.joinable()) {
//      thread.join();
//    }
//  }

//  std::cout << "Finished getBestInteraction " << approxMaxPosesToTry
//            << std::endl;
//  return bestInteraction;
// }

// int main(int argc, char **argv) {

//  cxxopts::Options options("PoseFilter - Filter and link poses to hopefully "
//                           "give the best pose set");
//  options.add_options()("f,file-prefix", "I/O File prefix",
//                        cxxopts::value<std::string>())(
//      "j,joint", "Favoured Joint", cxxopts::value<int>()->default_value("-1"))(
//      "l,linking", "linking (Chaining) Mode",
//      cxxopts::value<std::string>()->default_value("m"))(
//      "c,confRoot", "Conf path",
//      cxxopts::value<std::string>()->default_value("../../nao/home/"));

//  auto result = options.parse(argc, argv);

//  int jointNum = result["joint"].as<int>();
//  std::string inFileName = result["file-prefix"].as<std::string>();
//  std::string chainingModeStr = result["linking"].as<std::string>();
//  std::string confRoot = result["confRoot"].as<std::string>();

//  // 1= sum, 2 = multi
//  int chainingMode = 0;
//  if (chainingModeStr.compare("s") == 0) {
//    chainingMode = 1;
//  } else if (chainingModeStr.compare("m") == 0) {
//    chainingMode = 2;
//  }
//  std::cout << "Chaining Mode:\t" << chainingModeStr << std::endl;

//  /// Setup output file names
//  const std::string outCostsFileName(
//      inFileName + "_" + constants::ChainedPoseCostsFileName + "_" +
//      (jointNum >= 0 ? "j" + std::to_string(jointNum) : "generic"));
//  const std::string inCostsFileName(
//      inFileName + "_" + constants::FilteredPoseCostsFileName + "_" +
//      (jointNum >= 0 ? "j" + std::to_string(jointNum) : "generic"));
//  const std::string inPosesFileName(
//      inFileName + "_" + constants::FilteredPosesFileName + "_" +
//      (jointNum >= 0 ? "j" + std::to_string(jointNum) : "generic"));
//  const std::string outFilteredPosesFileName(
//      inFileName + "_" + constants::ChainedPosesFileName + "_" +
//      (jointNum >= 0 ? "j" + std::to_string(jointNum) : "generic"));
//  std::fstream outCostsFile =
//      std::fstream((outCostsFileName + ".txt"), std::ios::out);
//  std::fstream outFilteredPosesFile =
//      std::fstream((outFilteredPosesFileName + ".txt"), std::ios::out);

//  if (!outCostsFile.is_open() || !outFilteredPosesFile.is_open()) {
//    std::cerr << "output file creation failed. Aborting!!!" << std::endl;
//    exit(1);
//  }
//  std::fstream inCostFile =
//      std::fstream((inCostsFileName + ".bin"), std::ios::in | std::ios::binary);
//  std::fstream inPoseFile =
//      std::fstream((inPosesFileName + ".txt"), std::ios::in);
//  if (!inCostFile.is_open()) {
//    std::cerr << "input file creation failed. Aborting!!!" << std::endl;
//    exit(1);
//  }
//  // Sorted PoseCost vector
//  std::vector<PoseCost> sortedPoseCostVec;
//  /// populate sortedPoseCostVec;
//  {
//    size_t k = 0;

//    PoseCost tempPose(0, 0);
//    while (inCostFile.good()) {
//      //      std::cout << "read.." << k << "\n";
//      try {
//        tempPose.read(inCostFile);
//        sortedPoseCostVec.emplace_back(tempPose);
//        k++;

//      } catch (...) {
//        std::cout << k << ".,.." << std::endl;
//      }
//    }
//  }

//  /*
//   * Secondary sort run with poseInteractionns
//   */
//  //  std::vector<PoseInteraction> poseInteractionList;
//  //  size_t approxMaxPosesToTry = populatePoseInteractions(
//  //      poseInteractionList, sortedPoseCostVec, maxPoseInteractionCount);
//  //  poseToPoseInteractionSort(poseInteractionList);

//  // populate Id vec
//  PoseInteraction rootPoseInteraction =
//      getBestPoseInteraction(sortedPoseCostVec, MAX_THREADS);
//  std::vector<size_t> poseIdVec;
//  poseIdVec[0] = rootPoseInteraction.combinedId.first;
//  poseIdVec[1] = rootPoseInteraction.combinedId.second;

//  /// Chain the poses!!
//  if (chainingMode != 0) {
//    bool multiplicationMode = (chainingMode == 2);
//    std::cout << "Start "
//              << (multiplicationMode ? "Multiplication" : "Summation")
//              << " Chaining" << std::endl;

//    if (!poseToPoseChaining(
//            rootPoseInteraction, poseIdVec, sortedPoseCostVec,
//            multiplicationMode,
//            std::min(sortedPoseCostVec.size(), static_cast<size_t>(20)),
//            MAX_THREADS)) {
//      std::cerr << "Something went wrong in pose chaining" << std::endl;
//      return 1;
//    }
//  } else {
//    std::cout << "No Chaining" << std::endl;
//  }

//  /// Write the remaining buffers to file
//  {
// #if DO_COMMIT
//    /// Start mapping pose cost to poses
//    std::map<size_t, poseAndRawAngleT> poseMap;
//    poseAndRawAngleT tempPose;
//    while (inPoseFile.good()) {
//      inPoseFile >> tempPose;
//      poseMap.emplace(tempPose.pose.id, std::move(tempPose));
//    }
//    std::cout << "flushing all " << std::endl;
//    std::cout << "Commit to file.." << poseIdVec.size() << std::endl;
//    size_t iter = 0;
//    for (auto &id : poseIdVec) {
//      auto pose = poseMap.at(id);
//      try {
//        auto result =
//            std::find_if(sortedPoseCostVec.begin(), sortedPoseCostVec.end(),
//                         [&id](PoseCost &c) { return c.id == id; });
//        if (result != sortedPoseCostVec.end()) {
//          //                        poseList[i].pose.
//          outFilteredPosesFile << pose << "\n";
//          outCostsFile << "poseCost " << result->id << " " << result->jointCost
//                       << " " << result->jointInteractionCostVec.transpose()
//                       << "\n";
//          iter++;
//        }
//      } catch (std::exception e) {
//        std::cerr << e.what() << std::endl;
//        return 1;
//      }
//    }
//    outCostsFile.flush();
//    outFilteredPosesFile.flush();
//    std::cout << "commited % "
//              << (iter * 100) / static_cast<double>(sortedPoseCostVec.size())
//              << std::endl;
//    outCostsFile.close();
//    outFilteredPosesFile.close();
// #endif
//    std::cout << "All done\n" << std::endl;
//  }
//  return 0;
// }
