#include <algorithm>
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

#include <cxxopts/include/cxxopts.hpp>

#include "MiniConfigHandle.hpp"
#include "NaoPoseInfo.hpp"
#include "NaoStability.hpp"
#include "NaoTorsoPose.hpp"
#include "TUHHMin.hpp"
// #define DEBUG_CAM_OBS 1
#include "ObservationSensitivityProvider.hpp"

#define DO_COMMIT 1
#define WRITE_PARALLEL 1
#define DEBUG_FILE_COMMIT 1
#define DEBUG_IN_THREAD 1
#define ENABLE_PROGRESS 1

#include "utils.hpp"

static const size_t maxMemBuf = 5e9;
static std::atomic<size_t> maxBufSize(1);
const unsigned int MAX_THREADS = std::thread::hardware_concurrency();

void sensitivityTesterFunc(const ObservationModelConfig cfg,
                           std::istream &inputStream,
                           std::ostream &outputStream,
                           std::atomic<size_t> &iterations,
                           std::atomic<size_t> &sensitivityCount) {
  auto obs = ObservationSensitivityProvider::getSensitivityProvider(cfg);
  std::vector<PoseSensitivity<Vector3f>> sensitivityBuf;
  NaoPoseAndRawAngles<float> poseAndAngles;

  while (utils::JointsAndPosesStream::getNextPoseAndRawAngles(inputStream,
                                                              poseAndAngles)) {
    iterations++;
    std::vector<PoseSensitivity<Vector3f>> sensitivityOutput =
        obs.getSensitivities(poseAndAngles.angles,
                             poseAndAngles.pose.supportFoot);
    for (auto &i : sensitivityOutput) {
      if (i.getObservableCount() <= 0) {
        continue;
      }
      i.setId(poseAndAngles.pose.id);
      sensitivityBuf.push_back(i); // change for moving?
      sensitivityCount++;
    }
    /// if buffer is "well full", commit to output stream.
    if (sensitivityBuf.size() > maxBufSize) {
      utils::commitToStream(sensitivityBuf, outputStream);
    }
  }
  // flushing rest of the buffer
  utils::commitToStream(sensitivityBuf, outputStream);
}

int main(int argc, char **argv) {

  cxxopts::Options options("PoseFilter - Filter and link poses to hopefully "
                           "give the best pose set");
  options.add_options()("f,file-prefix", "I/O File prefix",
                        cxxopts::value<std::string>())(
      "c,confRoot", "Conf path",
      cxxopts::value<std::string>()->default_value("../../nao/home/"));

  auto result = options.parse(argc, argv);
  std::string inFileName = result["file-prefix"].as<std::string>();
  std::string confRoot = result["confRoot"].as<std::string>();

  const std::string outFileName(inFileName + "_ExtractedSensitivities");

  TUHH tuhhInstance(confRoot);

  Vector2f fc, cc, fov;

  tuhhInstance.config_.mount("Projection", "Projection.json",
                             ConfigurationType::HEAD);
  tuhhInstance.config_.get("Projection", "top_fc") >> fc;
  tuhhInstance.config_.get("Projection", "top_cc") >> cc;
  tuhhInstance.config_.get("Projection", "fov") >> fov;

  std::cout << "init" << std::endl;

  size_t usableThreads = 0;

  std::vector<std::fstream> inputPoseAndJointStreams;
  for (size_t i = 0; i < MAX_THREADS; i++) {
    std::string fileName =
        inFileName + "_GeneratedPoses_" + std::to_string(i) + ".txt";
    if (std::ifstream(fileName)) {
      inputPoseAndJointStreams.emplace_back(fileName, std::ios::in);
      usableThreads++;
    } else {
      break;
    }
  }

  // Update the buffer size per thread
  maxBufSize = maxMemBuf / (usableThreads * 500);
  std::cout << "Elem buf size per thread: " << maxBufSize
            << " of total mem: " << maxMemBuf << std::endl;
  /// Start the real threading..
  {
    std::vector<std::thread> threadList(usableThreads);
    std::vector<std::atomic<size_t>> iterCount(usableThreads);
    std::vector<std::atomic<size_t>> sensitivityCount(usableThreads);
    std::vector<std::fstream> outputFileList(usableThreads);

    /// observerModel config (to get grpound grid info, etc)
    Uni::Value obsModelConfig;
    if (!MiniConfigHandle::mountFile("configuration/cameraObsModelConf.json",
                                     obsModelConfig)) {
      std::cout << "couldn't open the conf file" << std::endl;
      exit(1);
    }

    Vector2i imSize;
    size_t maxGridPointsPerSide;
    size_t dimensionExtremum;
    float gridSpacing;
    obsModelConfig["imSize"] >> imSize;
    obsModelConfig["gridSpacing"] >> gridSpacing;
    maxGridPointsPerSide = static_cast<size_t>(std::ceil(
        static_cast<float>(obsModelConfig["gridSizeLength"].asDouble()) /
        gridSpacing));
    obsModelConfig["dimensionExtremum"] >> dimensionExtremum;

    const ObservationModelConfig cfg = {
        imSize,
        fc,
        cc,
        fov,
        {SENSOR_NAME::BOTTOM_CAMERA, SENSOR_NAME::TOP_CAMERA},
        dimensionExtremum,
        maxGridPointsPerSide,
        gridSpacing};

    for (unsigned int i = 0; i < usableThreads; i++) {
      outputFileList[i] = std::fstream(
          (outFileName + "Temp_" + std::to_string(i) + ".txt"), std::ios::out);
      if (!outputFileList[i].is_open()) {
        std::cerr << "output file creation failed. Aborting!!!" << std::endl;
        break;
      }

      threadList[i] = std::thread(
          sensitivityTesterFunc, cfg, std::ref(inputPoseAndJointStreams[i]),
          std::ref(outputFileList[i]), std::ref(iterCount[i]),
          std::ref(sensitivityCount[i]));
    }

/// Progress display
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
        std::this_thread::sleep_for(std::chrono::seconds(interval));
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
#if DO_COMMIT
    /// Write the remaining buffers to file
    std::cout << "flushing all " << std::endl;
    for (size_t i = 0; i < usableThreads; i++) {
      outputFileList[i].close();
    }

    std::cout << "Redistributing data" << std::endl;
    // redistribute the written lines.
    {
      const size_t linesPerFile =
          std::accumulate(sensitivityCount.begin(), sensitivityCount.end(),
                          (size_t)0) /
          (size_t)usableThreads;
      size_t outFileNum = 0;
      std::fstream inStream;
      std::fstream outStream = std::fstream(
          (outFileName + "_" + std::to_string(outFileNum) + ".txt"),
          std::ios::out);
      std::cout << "Write to file: " << outFileNum << std::endl;
      size_t counter = 0;
      size_t totalCounter = 0;

      for (size_t i = 0; i < usableThreads; i++) {
        std::cout << "Read file: " << i << std::endl;
        inStream.open((outFileName + "Temp_" + std::to_string(i) + ".txt"),
                      std::ios::in);
        if (!inStream) {
          continue;
        }
        std::string s;
        while (std::getline(inStream, s)) {
          outStream << s << std::endl;
          counter++;
          totalCounter++;
          if (counter > linesPerFile &&
              (outFileNum + 1) < usableThreads) // if last file, keep appending
                                                // to the same file.
          {
            outStream.close();
            outStream.clear();
            std::cout << "Finished writing to file: " << outFileNum
                      << " written: " << counter << std::endl;
            counter = 0;
            outFileNum++;
            std::cout << "Open write to file: " << outFileNum << std::endl;
            outStream.open(
                (outFileName + "_" + std::to_string(outFileNum) + ".txt"),
                std::ios::out);
          }
        }
        inStream.close();
        inStream.clear();
        // deleting the temp file.
        std::remove(
            (outFileName + "Temp_" + std::to_string(i) + ".txt").c_str());
      }
      std::cout << "Total redistributed lines: " << totalCounter << std::endl;
    }

#endif
  }

  return 0;
}
