#include <iostream>
#include <fstream>
#include <thread>

#include <Data/CameraMatrix.hpp>
#include <Modules/NaoProvider.h>
#include <Modules/Configuration/Configuration.h>
#include <Modules/Configuration/UnixSocketConfig.hpp>
#include <Modules/Poses.h>

#include <Tools/Storage/Image.hpp>
#include <Tools/Storage/Image422.hpp>
#include <Tools/Kinematics/KinematicMatrix.h>
#include <Hardware/RobotInterface.hpp>

#include "NaoStability.hpp"

#define DO_COMMIT 1
#define WRITE_PARALLEL 1
#define DEBUG_FILE_COMMIT 1
#define DEBUG_IN_THREAD 0

#include "utils.hpp"

typedef float dataT;
typedef JOINTS::JOINT jointT;
typedef std::vector<dataT> poseT;
typedef std::vector<poseT> poseListT;

const jointT JOINT_COUNT = JOINTS::JOINTS_MAX;

const unsigned int MAX_THREADS = std::thread::hardware_concurrency();
//2000MB(approx)  total buffer usage = BUFFER_SIZE *sizeof(int) * JOINT_COUNT * MAX_THREADS
const size_t totalBufferSizeInBytes = 2E9;
const size_t BUFFER_SIZE = totalBufferSizeInBytes / (sizeof(int) * static_cast<size_t>(JOINT_COUNT) * MAX_THREADS);

std::atomic<size_t> iterCount(0);
std::atomic<size_t> poseCount(0);

SupportPolygon supportPoly;

/**
 * Called at end of each update of joint angle
 */
inline bool poseCallback(poseT &pose)
{
    /// Where would the com be after setting these angles?
    KinematicMatrix com2torso = KinematicMatrix(Com::getCom(pose));

    KinematicMatrix lFoot2torso = ForwardKinematics::getLFoot(poseT(&pose[JOINTS::L_HIP_YAW_PITCH], &pose[JOINTS::L_ANKLE_ROLL]));
    KinematicMatrix rFoot2torso = ForwardKinematics::getRFoot(poseT(&pose[JOINTS::R_HIP_YAW_PITCH], &pose[JOINTS::R_ANKLE_ROLL]));

    bool isStable = supportPoly.isComWithinSupport(lFoot2torso, rFoot2torso, com2torso);
    return isStable;
}

/**
 * Iterate through a given joint, and recursively call the next joint..
 */

inline void getLimits(const jointT &jointIndex, dataT &minLim, dataT &maxLim, const poseT &pose)
{

    if (jointIndex == JOINTS::HEAD_PITCH)
    {
        // feed the current head yaw to get corresponding head pitch limits
        minLim = NaoProvider::minRangeHeadPitch(pose[JOINTS::HEAD_YAW]);
        maxLim = NaoProvider::maxRangeHeadPitch(pose[JOINTS::HEAD_YAW]);
    }
    else if (jointIndex == JOINTS::L_ANKLE_ROLL)
    {
        // feed the current head yaw to get corresponding head pitch limits
        minLim = NaoProvider::minRangeLAnkleRoll(pose[JOINTS::L_ANKLE_PITCH]);
        maxLim = NaoProvider::maxRangeLAnkleRoll(pose[JOINTS::L_ANKLE_PITCH]);
    }
    else if (jointIndex == JOINTS::R_ANKLE_ROLL)
    {
        // feed the current head yaw to get corresponding head pitch limits
        minLim = NaoProvider::minRangeRAnkleRoll(pose[JOINTS::R_ANKLE_PITCH]);
        maxLim = NaoProvider::maxRangeRAnkleRoll(pose[JOINTS::R_ANKLE_PITCH]);
    }
    else if(jointIndex == JOINTS::L_HIP_PITCH || jointIndex == JOINTS::R_HIP_PITCH)
    {
        minLim = NaoProvider::minRange(jointIndex);
        maxLim = NaoProvider::maxRange(jointIndex) ;
    }
    else
    {
        minLim = NaoProvider::minRange(jointIndex);
        maxLim = NaoProvider::maxRange(jointIndex);
    }
    // minLim = -1 * TO_RAD;
    // maxLim = 2 * TO_RAD;
}

inline void jointIterFuncWithLim(const jointT &jointIndex, const dataT &start, const dataT &end, dataT incrementInRad,
                                 poseT &pose, poseListT &poseList, const poseT &defaultPose, std::ostream &outStream,
                                 const bool &inclusiveMax);

inline void jointIterFunc(const jointT &jointIndex, dataT incrementInRad, poseT &pose,
                          poseListT &poseList, const poseT &defaultPose, std::ostream &outStream, const bool &inclusiveMax)
{
    dataT minLim, maxLim;
    getLimits(jointIndex, minLim, maxLim, pose);
    jointIterFuncWithLim(jointIndex, minLim, maxLim, incrementInRad, pose, poseList, defaultPose, outStream, inclusiveMax);
}

inline void jointIterFuncWithLim(const jointT &jointIndex, const dataT &start, const dataT &end, dataT incrementInRad,
                                 poseT &pose, poseListT &poseList, const poseT &defaultPose, std::ostream &outStream,
                                 const bool &inclusiveMax)
{
    // bool skip = false;
    // Skip arms, hands..
    if (jointIndex == JOINTS::L_SHOULDER_PITCH) // && jointIndex <= JOINTS::L_HAND
    {
        // assume accumilator is filled with defaults. (ready pose)
        // jump to yaw pitch.
        jointIterFunc(JOINTS::L_HIP_YAW_PITCH, incrementInRad, pose, poseList, defaultPose, outStream, true);
    }
    else if (jointIndex == JOINTS::R_SHOULDER_PITCH) // && jointIndex <= JOINTS::R_HAND
    {
        // After skipping RshoulderPitch to R_HAND, its end of the list!
        // jump to end of list
        // if (nextIndex == JOINT_COUNT)
        //     {
        iterCount++;
// #if DEBUG_IN_THREAD
        if (iterCount.load() % 100000 == 0)
        {
            std::lock_guard<std::mutex> lock(mtx_cout_);
            std::cout << "Iterations: " << iterCount.load() << " g. poses " << poseCount.load() << std::endl; // "\r";
        }
// #endif
        // std::cout << "came to end" << std::endl;
        if (poseCallback(pose))
        {
            poseCount++;
#if DO_COMMIT
            poseList.push_back(pose);
            if (poseList.size() > BUFFER_SIZE)
            {
#if !WRITE_PARALLEL
                while (!fileWriterReady.load())
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                }
#endif

                commitToStream<dataT>(poseList, outStream);
            }
#endif
        }
        // }
    }
    else if (jointIndex == JOINTS::R_HIP_YAW_PITCH)
    {
        // copy hip_yaw_pitch and jump to next
        pose[jointIndex] = pose[JOINTS::L_HIP_YAW_PITCH];
        jointIterFunc(JOINTS::R_HIP_ROLL, incrementInRad, pose, poseList, defaultPose, outStream, true);
    }
    // if (skip)
    // {
    //     const jointT nextIndex = static_cast<jointT>(jointIndex + 1);
    //     if (nextIndex < JOINT_COUNT)
    //     {
    //         jointIterFunc(nextIndex, incrementInRad, pose, poseList, defaultPose, outStream, true);
    //     }
    //     return;
    // }
    else
    {
        // this section will manipulate head, leg joints only. Others will be skipped.
        // End-of joints will be reached at a different place
        for (float j = start; j < end || (inclusiveMax ? j == end : false); j += incrementInRad)
        {
            // std::cout << "iter joint " << jointIndex << std::endl;
            /// skip validation for right hip yaw pitch as it is same as left hip yaw pitch
            pose[jointIndex] = j;
            const jointT nextIndex = static_cast<jointT>(jointIndex + 1);
            if (nextIndex < JOINT_COUNT)
            {
                jointIterFunc(nextIndex, incrementInRad, pose, poseList, defaultPose, outStream, true);
            }
// #if DEBUG_IN_THREAD
            /// If we get the "else", there is something wrong as end-of-list must be reached at above
            else
            {
                std::lock_guard<std::mutex> lock(mtx_cout_);
                std::cout << "SOMETHING IS WRONG!!!" << jointIndex << std::endl; // "\r";
            }
// #endif
        }
    }
}

// std::cout << "end of recursion" << std::endl;

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

    std::string outFileName((argc > 1 ? argv[1] : "out"));
    std::string confRoot((argc > 2 ? argv[2] : "/home/darshana/Documents/HULKs/nao/home/"));

    TUHH tuhhInstance(confRoot);

    int imageWidth = 640;
    int imageHeight = 480;

    CameraMatrix camMatrix;

    camMatrix.fc = Vector2f(imageWidth, imageHeight) / 2;
    camMatrix.cc = Vector2f(imageWidth, imageHeight) / 2;

    /// Pose Gen
    std::cout << "# Init for pose generation" << std::endl;

    const poseT readyPose(PARAMS::P_MAX);
    const dataT incrementInRad = 10 * TO_RAD; // 10 deg increment

    /// Start threading work.
    dataT minLimit, maxLimit;
    getLimits(static_cast<jointT>(0), minLimit, maxLimit, readyPose);
    maxLimit = 0; // TUHHNAO Look up limit.

    const int count = std::ceil(abs(maxLimit - minLimit) / (dataT)incrementInRad);
    const dataT splitVal = (dataT)count * incrementInRad / (dataT)MAX_THREADS;
    const size_t THREADS_USED = (splitVal > 0) ? MAX_THREADS : 1;

    /// PoseList vector and accum(pose) Vector
    std::vector<poseListT> poseListList(THREADS_USED);
    poseListT accumList(THREADS_USED);

    /// populate poseList and poseAccum
    for (auto &i : accumList)
    {
        i = Poses::getPose(Poses::READY);//poseT(JOINT_COUNT);
    }
    // for (auto &i : poseListList)
    // {
    //     i.reserve(BUFFER_SIZE); // = poseListT(BUFFER_SIZE);
    // }

    /// Start the real threading..
    {
        std::vector<std::thread> threadList(THREADS_USED);
        std::vector<std::fstream> outputFileList(THREADS_USED);

        for (unsigned int i = 0; i < THREADS_USED; i++)
        {
            outputFileList[i] = std::fstream((outFileName + "_" + std::to_string(i) + ".txt"), std::ios::out);
            if (!outputFileList[i].is_open())
            {
                std::cerr << "output file creation failed. Aborting!!!" << std::endl;
                break;
            }
            const bool lastIter = (i + 1 == THREADS_USED);

            dataT start = (minLimit + i * splitVal);
            dataT end = lastIter ? maxLimit : (minLimit + (i + 1) * splitVal);

            // (const jointT &jointIndex, const dataT &start, const dataT &end, dataT &incrementInRad,
            //                      poseT &pose, poseListT &poseList, const poseT &defaultPose, std::ostream &outStream,
            //                      const bool &inclusiveMax)
            std::cout << "Start, end, lastIter " << start << " " << end << " " << lastIter << std::endl;
            threadList[i] = std::thread(jointIterFuncWithLim, static_cast<jointT>(0), start, end, incrementInRad,
                                        std::ref(accumList[i]), std::ref(poseListList[i]), std::ref(readyPose),
                                        std::ref(outputFileList[i]), lastIter);
        }
        /// Join all :P
        for (auto &t : threadList)
        {
            if (t.joinable())
            {
                t.join();
            }
        }
        /// Write the remaining buffers to file
        std::cout << "flushing all " << std::endl;
#if DO_COMMIT
        for (size_t i = 0; i < THREADS_USED; i++)
        {
            commitToStream<dataT>(poseListList[i], outputFileList[i]);
            outputFileList[i].close();
        }
#endif
    }
    std::cout << "Tried " << iterCount.load() << " poses!" << std::endl;
    std::cout << "Found " << poseCount.load() << " good poses!" << std::endl;

    return 0;
}
