#include <iostream>
#include <fstream>
#include <thread>
#include <algorithm>

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
#include "NaoTorsoPose.hpp"
#include "NaoPoseInfo.hpp"

#define DO_COMMIT 1
#define WRITE_PARALLEL 1
#define DEBUG_FILE_COMMIT 1
#define DEBUG_IN_THREAD 1
#define ENABLE_PROGRESS 1

#include "utils.hpp"

typedef NaoTorsoPose::angleT dataT;
typedef PARAMS paramNameT;

// const dataT deltaTheta = 1; // 1 deg +-

typedef NaoTorsoPose::jointAnglesT rawAnglesT;
typedef std::vector<dataT> rawPoseT;

typedef std::vector<rawPoseT> rawPoseListT;

const unsigned int MAX_THREADS = std::thread::hardware_concurrency();

//2000MB(approx)  total buffer usage = BUFFER_SIZE *sizeof(int) * JOINT_COUNT * MAX_THREADS
const size_t totalBufferSizeInBytes = 200E6;
const size_t BUFFER_SIZE = totalBufferSizeInBytes / (sizeof(dataT) * static_cast<size_t>(PARAMS::P_MAX) * MAX_THREADS);

std::atomic<size_t> iterCount(0);
std::atomic<size_t> poseCount(0);

SupportPolygon supportPoly;

/**
 * Called at end of each update of joint angle
 */
inline bool poseCallback(rawAnglesT &pose, const SUPPORT_FOOT &supFoot)
{
    /// Where would the com be after setting these angles?
    KinematicMatrix com2torso = KinematicMatrix(Com::getCom(pose));

    KinematicMatrix lFoot2torso = ForwardKinematics::getLFoot(rawPoseT(&pose[JOINTS::L_HIP_YAW_PITCH], &pose[JOINTS::L_ANKLE_ROLL]));
    KinematicMatrix rFoot2torso = ForwardKinematics::getRFoot(rawPoseT(&pose[JOINTS::R_HIP_YAW_PITCH], &pose[JOINTS::R_ANKLE_ROLL]));

    bool isStable = supportPoly.isComWithinSupport(lFoot2torso, rFoot2torso, com2torso, supFoot);
    return isStable;
}

/**
 * Get limits for each tuning param
 */

inline void getLimits(const paramNameT &paramIndex, dataT &minLim, dataT &maxLim, dataT &increment, const rawPoseT &pose, const bool &resume)
{
    if (paramIndex >= static_cast<PARAMS>(PARAMS::P_MAX) || paramIndex < 0)
    {
        minLim = 0;
        maxLim = 0;
        increment = 0;
        std::cout << "param index out of bounds" << std::endl;
        return;
    }
    if (paramIndex == PARAMS::P_HEAD_YAW)
    {
        minLim = resume ? pose[PARAMS::P_HEAD_YAW] : -119.5;
        maxLim = 119.5;
        increment = 10;
        return;
    }
    else if (paramIndex == PARAMS::P_HEAD_PITCH)
    {
        // feed the current head yaw to get corresponding head pitch limits
        minLim = std::max((resume ? pose[PARAMS::P_HEAD_PITCH] : NaoProvider::minRangeHeadPitch(pose[PARAMS::P_HEAD_YAW]) / TO_RAD), 0.0f);
        maxLim = NaoProvider::maxRangeHeadPitch(pose[PARAMS::P_HEAD_YAW]) / TO_RAD;
        increment = 2;
        return;
    }
    else if (paramIndex == PARAMS::P_SUPPORT_FOOT)
    {
        minLim = resume ? pose[PARAMS::P_SUPPORT_FOOT] : static_cast<float>(SUPPORT_FOOT::SF_DOUBLE);
        maxLim = static_cast<float>(SUPPORT_FOOT::SF_DOUBLE);
        increment = 1;
        return;
    }

    SUPPORT_FOOT supFoot = static_cast<SUPPORT_FOOT>(pose[P_SUPPORT_FOOT]);
    // shift index by two and partition by pieces of 3
    int out = static_cast<int>(paramIndex - 3) / 3;
    // int remain = static_cast<int>(paramIndex) % 3;
    switch (out)
    {
    // torsoPosVector
    case 0:
        if (paramIndex == PARAMS::P_TORSO_POS_Z)
        {
            minLim = 0.2;
            maxLim = 0.45;
            increment = 0.025; // 2cm increment - 11
        }
        else
        {
            minLim = -0.1;
            maxLim = 0.1;
            increment = 0.02; // 2cm increment - 11
        }
        break;
    // torsoRotVec
    case 1:
        // minLim = -10;
        // maxLim = 10;
        minLim = 0;
        maxLim = 0;
        increment = 5; // 2 deg increment - 5
        break;
    // otherFootPosVec
    case 2:
        // keep origin of other foot always equal or upper than support foot.
        if (paramIndex == PARAMS::P_O_FOOT_POS_Z)
        {
            if (supFoot == SUPPORT_FOOT::SF_DOUBLE) // 1
            {
                minLim = 0.0;
                maxLim = 0.0;
            }
            else // 5
            {
                minLim = 0.0;
                maxLim = 0.1;
            }
        }
        else if (paramIndex == PARAMS::P_O_FOOT_POS_Y)
        {
            if (supFoot == SUPPORT_FOOT::SF_RIGHT) // 5
            {
                minLim = 0.0;
                maxLim = 0.10;
            }
            else // 5
            {
                minLim = -0.10;
                maxLim = 0.0;
            }
        }
        else // 11
        {
            minLim = -0.10;
            maxLim = 0.10;
        }
        increment = 0.02; // 2cm increment
        break;
    // otherFootRotVec
    case 3:
        // 1
        // if (supFoot == SUPPORT_FOOT::SF_DOUBLE && (paramIndex == PARAMS::P_O_FOOT_ROT_X || paramIndex == PARAMS::P_O_FOOT_ROT_Y))
        // {
        //     minLim = 0;
        //     maxLim = 0.1;
        //     increment = 0;
        // }
        // else
        // {
        //     minLim = -10;
        //     maxLim = 10;
        //     increment = 5; // 5 deg increment
        // }
        minLim = 0;
        maxLim = 0;
        increment = 0.1;
        break;
    };
    if (resume)
    {
        minLim = pose[paramIndex];
    }
}

inline void jointIterFuncWithLim(const paramNameT &jointIndex, const dataT &start, const dataT &end, dataT incrementInRad,
                                 rawPoseT &pose, rawPoseListT &poseList, const rawPoseT &defaultPose, std::ostream &outStream,
                                 const bool &inclusiveMax, bool &resume);

inline void jointIterFunc(const paramNameT &jointIndex, rawPoseT &pose,
                          rawPoseListT &poseList, const rawPoseT &defaultPose, std::ostream &outStream, const bool &inclusiveMax, bool &resume)
{
    dataT minLim, maxLim, increment;
    getLimits(jointIndex, minLim, maxLim, increment, pose, resume);
    jointIterFuncWithLim(jointIndex, minLim, maxLim, increment, pose, poseList, defaultPose, outStream, inclusiveMax, resume);
}

inline void jointIterFuncWithLim(const paramNameT &jointIndex, const dataT &start, const dataT &end, dataT incrementInRad,
                                 rawPoseT &pose, rawPoseListT &poseList, const rawPoseT &defaultPose, std::ostream &outStream,
                                 const bool &inclusiveMax, bool &resume)
{
    if (jointIndex != PARAMS::P_SUPPORT_FOOT)
    {
        // this section will manipulate head, leg joints only. Others will be skipped.
        // End-of joints will be reached at a different place
        for (dataT j = start; j < end || (inclusiveMax ? j == end : false); j += incrementInRad)
        {
            // std::cout << "iter joint " << jointIndex << std::endl;
            /// skip validation for right hip yaw pitch as it is same as left hip yaw pitch
            pose[jointIndex] = j; //abs(j) < incrementInRad ? j : 0; //if the current value is less than increment (near zero), then round off to zero.
            const paramNameT nextIndex = static_cast<paramNameT>(jointIndex + 1);
            if (nextIndex < PARAMS::P_MAX)
            {
                jointIterFunc(nextIndex, pose, poseList, defaultPose, outStream, true, resume);
            }
            else
            {
                // reached a full cycle, disable the resume hook..
                resume = false;

                iterCount++;
                SUPPORT_FOOT supportFootVal = static_cast<SUPPORT_FOOT>(pose[PARAMS::P_SUPPORT_FOOT]);
                /// Prefiltering.. Check for collisions and others here. :)
                /// First check, double foot..
                if (supportFootVal == SUPPORT_FOOT::SF_DOUBLE && (std::abs(pose[PARAMS::P_O_FOOT_POS_Z]) > __FLT_EPSILON__ ||
                                                                  std::abs(pose[PARAMS::P_O_FOOT_ROT_X]) > __FLT_EPSILON__ ||
                                                                  std::abs(pose[PARAMS::P_O_FOOT_ROT_Y]) > __FLT_EPSILON__))
                {
                    continue;
                }
                // TODO insert more checks here
                else
                {
                    rawAnglesT jointAngles = Poses::getPose(Poses::READY);

                    bool poseGenSuccess = NaoTorsoPose::getPose(jointAngles,
                                                                PoseUtils::getTorsoPosVFromParams(pose),
                                                                PoseUtils::getTorsoRotVFromParams(pose) * TO_RAD,
                                                                PoseUtils::getOtherFootPosVFromParams(pose),
                                                                PoseUtils::getOtherFootRotVFromParams(pose) * TO_RAD,
                                                                static_cast<SUPPORT_FOOT>(supportFootVal));
                    // if pose gen success and pose is statically stable
                    if (poseGenSuccess && poseCallback(pose, static_cast<SUPPORT_FOOT>(supportFootVal)))
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
                }
            }
        }
    }
    else
    {
        for (int supportFootIdx = static_cast<int>(SUPPORT_FOOT::SF_RIGHT);
             supportFootIdx < static_cast<int>(SUPPORT_FOOT::SF_NONE);
             supportFootIdx++)
        {
            pose[PARAMS::P_SUPPORT_FOOT] = supportFootIdx;

            const paramNameT nextIndex = static_cast<paramNameT>(jointIndex + 1);
            if (nextIndex < PARAMS::P_MAX)
            {
                jointIterFunc(nextIndex, pose, poseList, defaultPose, outStream, true, resume);
            }
#if DEBUG_IN_THREAD
            /// If we get the "else", there is something wrong as end-of-list must be reached at above
            else
            {
                std::lock_guard<std::mutex> lock(mtx_cout_);
                std::cout << "SOMETHING IS WRONG!!!" << jointIndex << std::endl; // "\r";
            }
#endif
        }
    }
}

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
    std::string outFileName((argc > 2 ? argv[2] : "out"));

    // std::vector<dataT> resumeVec;
    // std::static_assert(dataT == float); // splitter is float only :P

    bool resume = false; // (argc > 3 && std::string(argv[2]).compare("-r") == 0);
    // {
    //     std::string valString = argv[3];
    //     resumeVec = splitToNumbers(valString);
    //     if (resumeVec.size() == PARAMS::P_MAX)
    //     {
    //         resume = true;
    //         std::cout << "Resuming from values: " << valString << std::endl;
    //     }else{
    //         std::cout << "Resuming string parse failed: " << valString << std::endl;
    //     }
    // }

    TUHH tuhhInstance(confRoot);

    int imageWidth = 640;
    int imageHeight = 480;

    CameraMatrix camMatrix;

    camMatrix.fc = Vector2f(imageWidth, imageHeight) / 2;
    camMatrix.cc = Vector2f(imageWidth, imageHeight) / 2;

    /// Pose Gen
    std::cout << "# Init for pose generation" << std::endl;

    const rawPoseT readyPose(PARAMS::P_MAX);

    /// Start threading work.
    dataT minLimit, maxLimit, increment;
    getLimits(static_cast<paramNameT>(0), minLimit, maxLimit, increment, readyPose, resume);

    const int count = std::ceil(abs(maxLimit + 0.1 - minLimit) / (dataT)increment);

    const dataT splitVal = (dataT)count * increment / (dataT)MAX_THREADS;
    std::cout << "count " << count << " " << splitVal << std::endl;
    const size_t THREADS_USED = (splitVal > 1) ? MAX_THREADS : count;

    /// PoseList vector and accum(pose) Vector
    std::vector<rawPoseListT> poseListList(THREADS_USED);
    rawPoseListT accumList(THREADS_USED);

    /// populate poseList and poseAccum
    for (auto &i : accumList)
    {
        i = rawPoseT(PARAMS::P_MAX);
    }

    /// Start the real threading..
    {
        std::vector<std::thread> threadList(THREADS_USED);
        std::vector<std::fstream> outputFileList(THREADS_USED);
        // bool resumeFlags[THREADS_USED];
        for (unsigned int i = 0; i < THREADS_USED; i++)
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
            const bool lastIter = (i + 1 == THREADS_USED);

            dataT start = (minLimit + ((dataT)i * splitVal));
            dataT end = lastIter ? maxLimit : (minLimit + (i + 1) * splitVal);

            // (const paramNameT &jointIndex, const dataT &start, const dataT &end, dataT &incrementInRad,
            //                      poseT &pose, rawPoseListT &poseList, const poseT &defaultPose, std::ostream &outStream,
            //                      const bool &inclusiveMax)
            std::cout << "Start, end, lastIter " << start << " " << end << " " << lastIter << std::endl;
            threadList[i] = std::thread(jointIterFuncWithLim, static_cast<paramNameT>(0), start, end, increment,
                                        std::ref(accumList[i]), std::ref(poseListList[i]), std::ref(readyPose),
                                        std::ref(outputFileList[i]), lastIter, std::ref(resume));
        }
#if ENABLE_PROGRESS
        bool continueTicker = true;
        std::thread tTick = std::thread([&]() {
            int elapsed = 0;
            const size_t interval = 5;
            while (continueTicker)
            {
                {
                    std::lock_guard<std::mutex> lock(mtx_cout_);
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
        for (size_t i = 0; i < THREADS_USED; i++)
        {
            commitToStream<dataT>(poseListList[i], outputFileList[i]);
            outputFileList[i].close();
        }
#endif
    }
    std::cout << "Tried " << std::scientific << (double)iterCount.load() << " poses!" << std::endl;
    std::cout << "Found " << std::scientific << (double)poseCount.load() << " good poses!" << std::endl;

    return 0;
}
