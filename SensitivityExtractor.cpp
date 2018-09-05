#include <iostream>
#include <fstream>
#include <thread>
#include <algorithm>
#include <numeric>
#include <limits>

#include <Data/CameraMatrix.hpp>
#include <Modules/NaoProvider.h>
#include <Modules/Configuration/Configuration.h>
#include <Modules/Configuration/UnixSocketConfig.hpp>
#include <Modules/Poses.h>

#include <Tools/Storage/Image.hpp>
#include <Tools/Storage/Image422.hpp>
#include <Tools/Kinematics/KinematicMatrix.h>
#include <Hardware/RobotInterface.hpp>

#include "TUHHMin.hpp"
#include "NaoPoseInfo.hpp"
#include "NaoStability.hpp"
#include "NaoTorsoPose.hpp"
// #define DEBUG_CAM_OBS 1
#include "ObservationSensitivityProvider.hpp"

#define DO_COMMIT 1
#define WRITE_PARALLEL 1
#define DEBUG_FILE_COMMIT 1
#define DEBUG_IN_THREAD 1
#define ENABLE_PROGRESS 1

#include "utils.hpp"

const unsigned int MAX_THREADS = std::thread::hardware_concurrency();

void sensitivityTesterFunc(const ObservationModelConfig cfg, std::istream &inputStream, std::ostream &outputStream,
                           std::atomic<size_t> &iterations)
{
    auto obs = ObservationSensitivityProvider::getSensitivityProvider(cfg);

    NaoPoseAndRawAngles<float> poseAndAngles;
    while (utils::JointsAndPosesStream::getNextPoseAndRawAngles(inputStream, poseAndAngles))
    {
        iterations++;
        // std::cout << poseAndAngles << std::endl;
        std::vector<PoseSensitivity<Vector3f>> sensitivityOutput =
            obs.getSensitivities(poseAndAngles.angles, poseAndAngles.pose.supportFoot, {SENSOR_NAME::BOTTOM_CAMERA, SENSOR_NAME::TOP_CAMERA});
        // std::cout << sensitivityOutput.size() << " got this much?" << std::endl;
        for (auto &i : sensitivityOutput)
        {
            Vector3f val;
            bool obs;
            if (i.getObservableCount() <= 0)
            {
                continue;
            }
            outputStream << "SENS " << poseAndAngles.pose.id << " " << i.getSensorName() << " " << i.getDimensionCount() << " ";
            for (int j = 0; j < JOINTS::JOINT::JOINTS_MAX; j++)
            {
                i.getSensitivity(static_cast<JOINTS::JOINT>(j), val, obs);
                if (obs && val.norm() > __FLT_EPSILON__)
                {
                    outputStream << j << " ";
                    for (int k = 0; k < i.getDimensionCount(); k++)
                    {
                        outputStream << val(k) << " ";
                    }
                }
            }
            outputStream << std::endl;
        }
    }
}

int main(int argc, char **argv)
{
    std::string inFileName((argc > 1 ? argv[1] : "out"));
    std::string confRoot((argc > 2 ? argv[2] : "../../nao/home/"));

    const std::string outFileName(inFileName + "_sensOut");

    TUHH tuhhInstance(confRoot);

    Vector2f fc, cc, fov;

    tuhhInstance.config_.mount("Projection", "Projection.json", ConfigurationType::HEAD);
    tuhhInstance.config_.get("Projection", "top_fc") >> fc;
    tuhhInstance.config_.get("Projection", "top_cc") >> cc;
    tuhhInstance.config_.get("Projection", "fov") >> fov;

    Vector2i imSize(640, 480);

    size_t maxGridPointsPerSide = 25;

    std::cout << "init" << std::endl;

    size_t usableThreads = 0;

    std::vector<std::fstream> inputPoseAndJointStreams;
    for (size_t i = 0; i < MAX_THREADS; i++)
    {
        std::string fileName = inFileName + "_" + std::to_string(i) + ".txt";
        if (std::ifstream(fileName))
        {
            inputPoseAndJointStreams.emplace_back(fileName, std::ios::in);
            usableThreads++;
        }
        else
        {
            break;
        }
    }

    /// Start the real threading..
    {
        std::vector<std::thread> threadList(usableThreads);
        std::vector<std::atomic<size_t>> iterCount(usableThreads);
        std::vector<std::fstream> outputFileList(usableThreads);

        const ObservationModelConfig cfg = {imSize, fc, cc, fov, 1000, maxGridPointsPerSide, 0.05};

        // std::vector<ObservationSensitivity> obsSensitivities =
        //     ObservationSensitivityProvider::getSensitivityProviders(usableThreads, cfg);
        // bool resumeFlags[THREADS_USED];
        for (unsigned int i = 0; i < usableThreads; i++)
        {
            outputFileList[i] = std::fstream((outFileName + "_" + std::to_string(i) + ".txt"), std::ios::out);
            if (!outputFileList[i].is_open())
            {
                std::cerr << "output file creation failed. Aborting!!!" << std::endl;
                break;
            }

            threadList[i] = std::thread(sensitivityTesterFunc, cfg, std::ref(inputPoseAndJointStreams[i]),
                                        std::ref(outputFileList[i]), std::ref(iterCount[i]));
        }

        /// Progress display
#if ENABLE_PROGRESS
        bool continueTicker = true;
        std::thread tTick = std::thread([&]() {
            int elapsed = 0;
            const size_t interval = 5;
            while (continueTicker)
            {
                size_t iterSum = std::accumulate(iterCount.begin(), iterCount.end(), (size_t)0);
                if (elapsed % 100)
                {
                    std::lock_guard<std::mutex> lock(utils::mtx_cout_);
                    std::cout << "Elapsed: " << elapsed << "s Iterations: " << std::scientific << (double)iterSum << std::endl;
                }
                else
                {
                    std::stringstream t;
                    t << "Elapsed: " << elapsed << " ";
                    for (size_t i = 0; i < usableThreads; i++)
                    {
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
        for (auto &t : threadList)
        {
            if (t.joinable())
            {
                t.join();
            }
        }
#if ENABLE_PROGRESS
        continueTicker = false;
        if (tTick.joinable())
        {
            tTick.join();
        }
#endif
#if DO_COMMIT
        /// Write the remaining buffers to file
        std::cout << "flushing all " << std::endl;
        for (size_t i = 0; i < usableThreads; i++)
        {
            outputFileList[i].close();
        }
#endif
    }

    return 0;
}
