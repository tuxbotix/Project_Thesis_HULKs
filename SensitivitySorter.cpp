#include <iostream>
#include <fstream>
#include <thread>
#include <algorithm>
#include <numeric>

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
#include "ObservationSensitivityProvider.hpp"

#define DO_COMMIT 1
#define WRITE_PARALLEL 1
#define DEBUG_FILE_COMMIT 1
#define DEBUG_IN_THREAD 1
#define ENABLE_PROGRESS 1

#include "utils.hpp"

// typedef NaoTorsoPose::angleT dataT;
// typedef PARAMS paramNameT;

// // const dataT deltaTheta = 1; // 1 deg +-

// typedef NaoTorsoPose::jointAnglesT rawAnglesT;
// typedef std::vector<dataT> rawPoseT;

// typedef std::vector<rawPoseT> rawPoseListT;

// const unsigned int MAX_THREADS = std::thread::hardware_concurrency();

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

int main(int argc, char **argv)
{
    std::string confRoot((argc > 1 ? argv[1] : "../../nao/home/"));
    std::string inFileName((argc > 2 ? argv[2] : "out"));

    TUHH tuhhInstance(confRoot);

    Vector2f fc, cc, fov;

    tuhhInstance.config_.mount("Projection", "Projection.json", ConfigurationType::HEAD);
    tuhhInstance.config_.get("Projection", "top_fc") >> fc;
    tuhhInstance.config_.get("Projection", "top_cc") >> cc;
    // tuhhInstance.config_.get("Projection", "top_fov") >> fov;

    Vector2i imSize(640, 480);

    /// Pose Gen
    std::cout << "# Init for Sensitivity Sorting" << std::endl;

    Vector2f i(10,40);
    Eigen::Rotation2D<float> rot(20*TO_RAD);
    std::cout<<(rot * i)<<std::endl;
    //     const rawPoseT readyPose(PARAMS::P_MAX);

    //     /// Start threading work.
    //     dataT minLimit, maxLimit, increment;
    //     getLimits(static_cast<paramNameT>(0), minLimit, maxLimit, increment, readyPose, resume);

    //     const unsigned int count = std::ceil(abs(maxLimit + 0.1 - minLimit) / (dataT)increment);

    //     const dataT splitVal = count >= MAX_THREADS ? (std::floor((dataT)count / (dataT)MAX_THREADS) * increment) : (increment);
    //     std::cout << "count " << count << " " << splitVal << std::endl;
    //     const size_t THREADS_USED = (splitVal > 1) ? MAX_THREADS : count;

    //     /// PoseList vector and accum(pose) Vector
    //     std::vector<rawPoseListT> poseListList(THREADS_USED);
    //     rawPoseListT accumList(THREADS_USED);

    //     /// populate poseList and poseAccum
    //     for (auto &i : accumList)
    //     {
    //         i = rawPoseT(PARAMS::P_MAX);
    //     }

    //     /// Start the real threading..
    //     {
    //         std::vector<std::thread> threadList(THREADS_USED);
    //         std::vector<std::fstream> outputFileList(THREADS_USED);
    //         std::vector<SupportPolygon> supportPolyList(THREADS_USED);
    //         std::vector<ObservationSensitivity> obsSensitivities =
    //             ObservationSensitivityProvider::getSensitivityProviders(THREADS_USED, imSize, fc, cc, fov, 0.05);
    //         // bool resumeFlags[THREADS_USED];
    //         for (unsigned int i = 0; i < THREADS_USED; i++)
    //         {
    //             // if(resume){
    //             //     outputFileList[i] = std::fstream((outFileName + "_" + std::to_string(i) + ".txt"), std::ios::in);
    //             //     if(outputFileList[i].is_open()){

    //             //     }
    //             // }

    //             outputFileList[i] = std::fstream((outFileName + "_" + std::to_string(i) + ".txt"), std::ios::out);
    //             if (!outputFileList[i].is_open())
    //             {
    //                 std::cerr << "output file creation failed. Aborting!!!" << std::endl;
    //                 break;
    //             }
    //             const bool lastIter = (i + 1 == THREADS_USED);

    //             dataT start = (minLimit + ((dataT)i * splitVal));
    //             dataT end = lastIter ? maxLimit : (minLimit + (i + 1) * splitVal);

    //             // (const paramNameT &jointIndex, const dataT &start, const dataT &end, dataT &incrementInRad,
    //             //                      poseT &pose, rawPoseListT &poseList, const poseT &defaultPose, std::ostream &outStream,
    //             //                      const bool &inclusiveMax)
    //             std::cout << "Start, end, lastIter " << start << " " << end << " " << lastIter << std::endl;
    //             threadList[i] = std::thread(jointIterFuncWithLim, static_cast<paramNameT>(0), start, end, increment,
    //                                         std::ref(accumList[i]), std::ref(poseListList[i]), std::ref(readyPose),
    //                                         std::ref(outputFileList[i]), std::ref(supportPolyList[i]), lastIter, std::ref(resume));
    //         }
    // #if ENABLE_PROGRESS
    //         bool continueTicker = true;
    //         std::thread tTick = std::thread([&]() {
    //             int elapsed = 0;
    //             const size_t interval = 5;
    //             while (continueTicker)
    //             {
    //                 {
    //                     std::lock_guard<std::mutex> lock(mtx_cout_);
    //                     std::cout << "Elapsed: " << elapsed << "s Iterations: " << std::scientific << (double)iterCount.load() << " g. poses " << poseCount.load() << std::endl; // "\r";
    //                 }
    //                 elapsed += interval;
    //                 std::this_thread::sleep_for(std::chrono::seconds(interval)); // 100Hz -> 10ms
    //             }
    //         });
    // #endif
    //         /// Join all :P
    //         for (auto &t : threadList)
    //         {
    //             if (t.joinable())
    //             {
    //                 t.join();
    //             }
    //         }
    // #if ENABLE_PROGRESS
    //         continueTicker = false;
    //         if (tTick.joinable())
    //         {
    //             tTick.join();
    //         }
    // #endif
    //         /// Write the remaining buffers to file
    //         std::cout << "flushing all " << std::endl;
    // #if DO_COMMIT
    //         for (size_t i = 0; i < THREADS_USED; i++)
    //         {
    //             commitToStream<dataT>(poseListList[i], outputFileList[i]);
    //             outputFileList[i].close();
    //         }
    // #endif
    //     }
    //     std::cout << "Tried " << std::scientific << (double)iterCount.load() << " poses!" << std::endl;
    //     std::cout << "Found " << std::scientific << (double)poseCount.load() << " good poses!" << std::endl;

    return 0;
}
