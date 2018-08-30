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

#include "NaoPoseInfo.hpp"
#include "NaoStability.hpp"
#include "NaoTorsoPose.hpp"
#define DEBUG_CAM_OBS 1
#include "ObservationSensitivityProvider.hpp"

#define DO_COMMIT 1
#define WRITE_PARALLEL 1
#define DEBUG_FILE_COMMIT 1
#define DEBUG_IN_THREAD 1
#define ENABLE_PROGRESS 0

#include "utils.hpp"

// typedef NaoTorsoPose::angleT dataT;
// typedef PARAMS paramNameT;

// // const dataT deltaTheta = 1; // 1 deg +-

const unsigned int MAX_THREADS = std::thread::hardware_concurrency();

/**
 * Minimal TUHH class impl. in order to use configuration
 * This is a rather hacky way, but doesn't need touching the actual belly of the beast xD
 */
class Configuration;
class TUHH
{

  public:
    Configuration config_;
    TUHH(std::string fileRoot) : config_(fileRoot)
    {
        NaoInfo info;
        info.bodyVersion = NaoVersion::V5;
        info.headVersion = NaoVersion::V5;
        info.bodyName = "default";
        info.headName = "default";

        config_.setLocationName("default");
        config_.setNaoHeadName(info.headName);
        config_.setNaoBodyName(info.bodyName);

        NaoProvider::init(config_, info);
        std::cout << "initialize TUHH" << std::endl;
        std::cout << Poses::init(fileRoot) << std::endl;
    }
};

class JointsAndPosesStream
{
  public:
    static bool getNextPoseAndRawAngles(std::istream &inputPoseFile, NaoPoseAndRawAngles<float> &val)
    {
        if (inputPoseFile.good())
        {
            std::string poseStr;
            std::getline(inputPoseFile, poseStr);
            if (inputPoseFile.good())
            {
                std::stringstream line(poseStr);
                line >> val;
                return val.isGood();
            }
        }
        return false;
    }
};

typedef int16_t dataOutType;

void sensitivityTesterFunc(ObservationSensitivity &obs, std::istream &inputStream, std::ostream &outputStream)
{
    NaoPoseAndRawAngles<float> poseAndAngles;
    while (JointsAndPosesStream::getNextPoseAndRawAngles(inputStream, poseAndAngles))
    {
        // std::cout << poseAndAngles << std::endl;
        std::vector<PoseSensitivity<Vector3f>> sensitivityOutput =
            obs.getSensitivities(poseAndAngles.angles, poseAndAngles.pose.supportFoot, {SENSOR_NAME::BOTTOM_CAMERA, SENSOR_NAME::TOP_CAMERA});
        // std::cout << sensitivityOutput.size() << " got this much?" << std::endl;
        for (auto &i : sensitivityOutput)
        {
            Vector3f val;
            bool obs;
            outputStream << "SENS " << poseAndAngles.pose.id << " " << i.getSensorName() << " " << i.getDimensionCount() << " ";
            for (int j = 0; j < JOINTS::JOINT::JOINTS_MAX; j++)
            {
                i.getSensitivity(static_cast<JOINTS::JOINT>(j), val, obs);
                if (obs)
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

    std::string confRoot((argc > 1 ? argv[1] : "../../nao/home/"));
    std::string inFileName((argc > 2 ? argv[2] : "out"));
    const std::string outFileName(inFileName + "_sensOut");

    TUHH tuhhInstance(confRoot);

    Vector2f fc, cc, fov;

    tuhhInstance.config_.mount("Projection", "Projection.json", ConfigurationType::HEAD);
    tuhhInstance.config_.get("Projection", "top_fc") >> fc;
    tuhhInstance.config_.get("Projection", "top_cc") >> cc;
    tuhhInstance.config_.get("Projection", "fov") >> fov;

    Vector2i imSize(640, 480);

    size_t maxGridPointsPerSide = 15;

    std::cout << "init" << std::endl;
    // std::vector<ObservationSensitivity> sensitivitues = ObservationSensitivityProvider::getSensitivityProviders(
    //     1, imSize, fc, cc, fov, maxGridPointsPerSide, 0.05);

    size_t usableThreads = 0;

    // JointsAndPosesStream poseJointSource(inFileName + "_" + std::to_string(0) + ".txt");

    // NaoPoseAndRawAngles<float> val;
    // if (poseJointSource.getNextPoseAndRawAngles(val))
    // {
    //     std::cout << val << std::endl;
    // }
    // else
    // {
    //     std::cout << "Reading pose and angles failed" << std::endl;
    // }

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
        std::vector<std::fstream> outputFileList(usableThreads);
        std::vector<ObservationSensitivity> obsSensitivities =
            ObservationSensitivityProvider::getSensitivityProviders(usableThreads, imSize, fc, cc, fov, 1000, maxGridPointsPerSide, 0.05);
        // bool resumeFlags[THREADS_USED];
        for (unsigned int i = 0; i < usableThreads; i++)
        {
            // if(resume){
            //     outputFileList[i] = std::fstream((outFileName + "_" + std::to_string(i) + ".txt"), std::ios::in);
            //     if(outputFileList[i].is_open()){

            //     }
            // }

            outputFileList[i] = std::fstream((outFileName + "_" + std::to_string(i) + ".txt"), std::ios::out);
            if (!outputFileList[i].is_open())
            {
                std::cerr << "output file creation failed. Aborting!!!" << std::endl;
                break;
            }

            threadList[i] = std::thread(sensitivityTesterFunc, std::ref(obsSensitivities[i]), std::ref(inputPoseAndJointStreams[i]),
                                        std::ref(outputFileList[i]));
        }
#if ENABLE_PROGRESS
        bool continueTicker = true;
        std::thread tTick = std::thread([&]() {
            int elapsed = 0;
            const size_t interval = 5;
            while (continueTicker)
            {
                {
                    std::lock_guard<std::mutex> lock(utils::mtx_cout_);
                    std::cout << "Elapsed: " << elapsed << "s Iterations: " << std::scientific << (double)iterCount.load() << " g. poses " << poseCount.load() << std::endl; // "\r";
                }
                elapsed += interval;
                std::this_thread::sleep_for(std::chrono::seconds(interval)); // 100Hz -> 10ms
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
        /// Write the remaining buffers to file
        std::cout << "flushing all " << std::endl;
#if DO_COMMIT
        for (size_t i = 0; i < usableThreads; i++)
        {
            // utils::commitToStream<rawPoseT>(poseListList[i], outputFileList[i]);
            outputFileList[i].close();
        }
#endif
    }

    return 0;
}
