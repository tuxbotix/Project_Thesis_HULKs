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

#include "NaoStability.hpp"
#include "NaoTorsoPose.hpp"
#include "NaoPoseInfo.hpp"
#include "ObservationSensitivityProvider.hpp"

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

struct HeadYawPitch
{
    dataT yaw;
    dataT pitch;
    HeadYawPitch(const dataT &y, const dataT &p) : yaw(y), pitch(p)
    {
    }
};

const unsigned int MAX_THREADS = std::thread::hardware_concurrency();

//2000MB(approx)  total buffer usage = BUFFER_SIZE *sizeof(int) * JOINT_COUNT * MAX_THREADS
const size_t totalBufferSizeInBytes = 200E6;
const size_t BUFFER_SIZE = totalBufferSizeInBytes / (sizeof(dataT) * static_cast<size_t>(PARAMS::P_MAX) * MAX_THREADS);

std::atomic<size_t> poseCount(0);

/**
 * Called at end of each update of joint angle
 */
inline bool poseCallback(rawAnglesT &pose, const SUPPORT_FOOT &supFoot, const SupportPolygon &supportPoly)
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
inline void getSupportFootLimits(SUPPORT_FOOT &minLim, SUPPORT_FOOT &maxLim, SUPPORT_FOOT &increment)
{
    //TODO fix this..
    minLim = SUPPORT_FOOT::SF_DOUBLE;
    maxLim = SUPPORT_FOOT::SF_DOUBLE;
    increment = static_cast<SUPPORT_FOOT>(1);
}
inline void getHeadYawLimits(dataT &minLim, dataT &maxLim, dataT &increment) // 8
{
    // Instead of the full range, i'll limit..
    // minLim = -72;
    // maxLim = 72;
    // increment = 18;
    minLim = -1;
    maxLim = 1;
    increment = 1;
    return;
}
inline void getHeadPitchLimits(const dataT &headYaw, dataT &minLim, dataT &maxLim, dataT &increment) // 19
{
    // minLim = std::max(NaoProvider::minRangeHeadPitch(headYaw / TO_RAD), 0.0f);
    // maxLim = NaoProvider::maxRangeHeadPitch(headYaw) / TO_RAD;
    minLim = 0;
    maxLim = 2;
    increment = 2;
    return;
}
inline void getLimits(const paramNameT &paramIndex, const SUPPORT_FOOT &supFoot, dataT &minLim, dataT &maxLim, dataT &increment)
{
    if (paramIndex >= static_cast<PARAMS>(PARAMS::P_MAX) || paramIndex < 0)
    {
        minLim = 0;
        maxLim = 0;
        increment = 0;
        std::cout << "param index out of bounds" << std::endl;
        return;
    }
    else if (paramIndex == PARAMS::P_HEAD_YAW || paramIndex == PARAMS::P_HEAD_PITCH || paramIndex == PARAMS::P_SUPPORT_FOOT)
    {
        std::lock_guard<std::mutex> lock(mtx_cout_);
        std::cout << "SOMETHING IS WRONG, This method should not call head or support foot!!!" << paramIndex << std::endl; // "\r";
        return;
    }

    switch (paramIndex)
    {
    // torsoPosVector
    case PARAMS::P_TORSO_POS_X:
    case PARAMS::P_TORSO_POS_Y:
    {
        // minLim = -0.1;
        // maxLim = 0.1;
        // increment = 0.02; // 2cm increment - 11
        minLim = 0;
        maxLim = 0;
        increment = 0.02; // 2cm increment - 11
        break;
    }
    case PARAMS::P_TORSO_POS_Z:

    {
        minLim = 0.2;
        maxLim = 0.45;
        increment = 0.025; // 2cm increment - 11
        break;
    }
    case PARAMS::P_TORSO_ROT_X:
    case PARAMS::P_TORSO_ROT_Y:
    // {
    //     minLim = -10;
    //     maxLim = 10;
    //     increment = 5; // 2 deg increment - 5
    //     break;
    // }
    case PARAMS::P_TORSO_ROT_Z:
    {
        minLim = 0;
        maxLim = 0;
        increment = 5; // 2 deg increment - 5
        break;
    }
    case PARAMS::P_O_FOOT_POS_X:
    {
        minLim = -0.10;
        maxLim = 0.10;
        increment = 0.02; // 2cm increment
        break;
    }
    // keep origin of other foot always equal or upper than support foot.
    case PARAMS::P_O_FOOT_POS_Z:
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
        increment = 0.04; // 2cm increment
    }
    case PARAMS::P_O_FOOT_POS_Y:
    {
        if (supFoot == SUPPORT_FOOT::SF_RIGHT) // 5
        {
            minLim = 0.0;
            maxLim = 0.10;
        }
        else // 5
        {
            minLim = -0.10;
            maxLim = -0.10; //0.0;
        }
        increment = 0.02; // 2cm increment
        break;
    }
    case PARAMS::P_O_FOOT_ROT_X:
    case PARAMS::P_O_FOOT_ROT_Y:
    case PARAMS::P_O_FOOT_ROT_Z:
    {
        minLim = 0;
        maxLim = 0;
        increment = 5; // 2 deg increment - 5
        break;
    }
    default:
    {
        {
            std::lock_guard<std::mutex> lock(mtx_cout_);
            std::cout << "Default should not be reached.. method should not call head or support foot!!!" << paramIndex << std::endl; // "\r";
        }
        minLim = 0;
        maxLim = 0;
        increment = 0.1;
        break;
    }
    };
}

void jointIterFuncWithLim(const paramNameT &jointIndex, const dataT &start, const dataT &end, dataT incrementInRad,
                          rawPoseT &pose, rawPoseListT &poseList, const rawPoseT &defaultPose, std::ostream &outStream,
                          const SupportPolygon &supportPoly, std::atomic<size_t> &iterCount, const bool &inclusiveMax);

inline void jointIterFunc(const paramNameT &jointIndex, rawPoseT &pose,
                          rawPoseListT &poseList, const rawPoseT &defaultPose, std::ostream &outStream, const SupportPolygon &supportPoly,
                          std::atomic<size_t> &iterCount, const bool &inclusiveMax)
{
    if (jointIndex == PARAMS::P_SUPPORT_FOOT)
    {
        std::lock_guard<std::mutex> lock(mtx_cout_);
        std::cout << "SOMETHING IS WRONG, This should not be called for head or supprotfoot iterations!!!" << jointIndex << std::endl; // "\r";
        return;
    }
    dataT minLim, maxLim, increment;
    if (jointIndex == PARAMS::P_HEAD_YAW)
    {
        getHeadYawLimits(minLim, maxLim, increment);
    }
    else if (jointIndex == PARAMS::P_HEAD_PITCH)
    {
        getHeadPitchLimits(pose[PARAMS::P_HEAD_YAW], minLim, maxLim, increment);
    }
    else
    {
        getLimits(jointIndex, static_cast<SUPPORT_FOOT>(pose[PARAMS::P_SUPPORT_FOOT]), minLim, maxLim, increment);
    }
    jointIterFuncWithLim(jointIndex, minLim, maxLim, increment, pose, poseList, defaultPose, outStream, supportPoly, iterCount, inclusiveMax);
}

void jointIterFuncWithLim(const paramNameT &jointIndex, const dataT &start, const dataT &end, dataT incrementInRad,
                          rawPoseT &pose, rawPoseListT &poseList, const rawPoseT &defaultPose, std::ostream &outStream,
                          const SupportPolygon &supportPoly, std::atomic<size_t> &iterCount, const bool &inclusiveMax)
{
    if (jointIndex >= PARAMS::P_MAX)
    {
        std::lock_guard<std::mutex> lock(mtx_cout_);
        std::cout << "SOMETHING IS WRONG, exceeding limits!!!" << jointIndex << std::endl; // "\r";
        return;
    }
    if (jointIndex != PARAMS::P_SUPPORT_FOOT)
    {
        // this section will manipulate head, leg joints only. Others will be skipped.
        // End-of joints will be reached at a different place
        for (dataT j = start; (inclusiveMax ? j <= end : j < end); j += incrementInRad)
        {
            // std::cout << "iter joint " << jointIndex << std::endl;
            /// skip validation for right hip yaw pitch as it is same as left hip yaw pitch
            pose[jointIndex] = j; //abs(j) < incrementInRad ? j : 0; //if the current value is less than increment (near zero), then round off to zero.
            const paramNameT nextIndex = static_cast<paramNameT>(jointIndex + 1);
            if (nextIndex < PARAMS::P_MAX)
            {
                jointIterFunc(nextIndex, pose, poseList, defaultPose, outStream, supportPoly, iterCount, true);
            }
            else
            {
                iterCount++;
                SUPPORT_FOOT supportFootVal = static_cast<SUPPORT_FOOT>(pose[PARAMS::P_SUPPORT_FOOT]);
                // Prefiltering.. Check for collisions and others here. :)
                // First check, double foot..
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
                    if (poseGenSuccess && poseCallback(pose, static_cast<SUPPORT_FOOT>(supportFootVal), supportPoly))
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
#if DEBUG_IN_THREAD
    else
    {
        /// If we get the "else", there is something wrong as end-of-list must be reached at above
        std::lock_guard<std::mutex> lock(mtx_cout_);
        std::cout << "SOMETHING IS WRONG!!!" << jointIndex << std::endl; // "\r";
    }
#endif
}

void iterWithHeadYawPitchSupportFoot(
    const size_t start, const size_t end, std::vector<HeadYawPitch> &headYawPitchList, SUPPORT_FOOT supportFoot, rawPoseT &accum, rawPoseListT &poseList, const rawPoseT &readyPose, std::ostream &outStream,
    const SupportPolygon &supportPoly, std::atomic<size_t> &iterCounter)
{
    for (size_t iter = start; iter < end; iter++)
    {
        accum[PARAMS::P_HEAD_YAW] = headYawPitchList[iter].yaw;
        accum[PARAMS::P_HEAD_PITCH] = headYawPitchList[iter].pitch;
        accum[PARAMS::P_SUPPORT_FOOT] = supportFoot;
        jointIterFunc(PARAMS::P_TORSO_POS_X, accum, poseList, readyPose, outStream, supportPoly, iterCounter, true);
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
    std::string supportFootName((argc > 3 ? argv[3] : "d"));

    SUPPORT_FOOT supportFoot = SUPPORT_FOOT::SF_DOUBLE;
    if (supportFootName.compare("l") == 0)
    {
        supportFoot = SUPPORT_FOOT::SF_LEFT;
    }
    else if (supportFootName.compare("r") == 0)
    {
        supportFoot = SUPPORT_FOOT::SF_RIGHT;
    }
    else
    {
        std::cout << "Going for double foot." << std::endl;
    }

    TUHH tuhhInstance(confRoot);
    /// Pose Gen
    std::cout << "# Init for pose generation" << std::endl;

    const rawPoseT readyPose(PARAMS::P_MAX);

    std::vector<HeadYawPitch> headYawPitchList;
    {
        float minLimit, maxLimit, increment;
        getHeadYawLimits(minLimit, maxLimit, increment);
        for (dataT yaw = minLimit; yaw <= maxLimit; yaw += increment)
        {
            float minLimit2, maxLimit2, increment2;
            getHeadPitchLimits(yaw, minLimit2, maxLimit2, increment2);
            for (dataT pitch = minLimit2; pitch <= maxLimit2; pitch += increment2)
            {
                headYawPitchList.emplace_back(yaw, pitch);
            }
        }
    }
    {
        size_t count = headYawPitchList.size();
        float minLimit, maxLimit, increment;
        // SUPPORT_FOOT minLimitSF, maxLimitSF, incrementSF;
        for (int i = static_cast<int>(PARAMS::P_TORSO_POS_X); i < static_cast<int>(paramNameT::P_MAX); i++)
        {
            getLimits(static_cast<paramNameT>(i), supportFoot, minLimit, maxLimit, increment);
            if ((maxLimit - minLimit) < 0)
            {
                std::cout << "min > max???" << std::endl;
            }
            count *= std::floor((maxLimit - minLimit) / increment) + 1;
            // }
        }
        std::cout << "Max iters -> " << std::scientific << (double)count << std::endl;
    }
    /// Start threading work.
    // size_t headPitchYawMin = 0;
    size_t headPitchYawSize = headYawPitchList.size();

    const dataT splitVal = headPitchYawSize >= MAX_THREADS ? (headPitchYawSize / MAX_THREADS) : 1;
    std::cout << "count " << std::defaultfloat << headPitchYawSize << " " << splitVal << std::endl;
    const size_t THREADS_USED = (splitVal > 1) ? MAX_THREADS : headPitchYawSize;

    /// PoseList vector and accum(pose) Vector
    std::vector<rawPoseListT> poseListList(THREADS_USED);
    rawPoseListT accumList(THREADS_USED);

    /// populate poseList and poseAccum
    for (auto &i : accumList)
    {
        i = rawPoseT(PARAMS::P_MAX);
        i[PARAMS::P_HEAD_YAW] = headYawPitchList[0].yaw;
        i[PARAMS::P_HEAD_PITCH] = headYawPitchList[0].pitch;
        i[PARAMS::P_SUPPORT_FOOT] = supportFoot;
    }

    std::vector<std::atomic<size_t>> iterCount(THREADS_USED);
    /// Start the real threading..
    {
        std::vector<std::thread> threadList(THREADS_USED);
        std::vector<std::fstream> outputFileList(THREADS_USED);
        std::vector<SupportPolygon> supportPolyList(THREADS_USED);
        // bool resumeFlags[THREADS_USED];
        for (size_t i = 0; i < THREADS_USED; i++)
        {
            outputFileList[i] = std::fstream((outFileName + "_" + std::to_string(i) + ".txt"), std::ios::out);
            if (!outputFileList[i].is_open())
            {
                std::cerr << "output file creation failed. Aborting!!!" << std::endl;
                break;
            }
            const bool lastIter = (i + 1 == THREADS_USED);

            size_t start = i * splitVal;
            size_t end = lastIter ? headPitchYawSize : (i + 1) * splitVal;
            // &jointIndex, rawPoseT &pose,
            //                           rawPoseListT &poseList, const rawPoseT &defaultPose, std::ostream &outStream, const SupportPolygon &supportPoly,
            //                           std::atomic<size_t> &iterCount, const bool &inclusiveMax)
            std::cout << "Start, end, lastIter " << start << " " << end << " " << lastIter << std::endl;
            threadList[i] = std::thread(iterWithHeadYawPitchSupportFoot, start, end, std::ref(headYawPitchList), supportFoot, std::ref(accumList[i]), std::ref(poseListList[i]),
                                        std::ref(readyPose), std::ref(outputFileList[i]), std::ref(supportPolyList[i]), std::ref(iterCount[i]));
        }
#if ENABLE_PROGRESS
        bool continueTicker = true;
        std::thread tTick = std::thread([&]() {
            int elapsed = 0;
            const size_t interval = 5;
            while (continueTicker)
            {
                size_t iterSum = std::accumulate(iterCount.begin(), iterCount.end(), 0);
                if (elapsed % 100)
                {
                    std::lock_guard<std::mutex> lock(mtx_cout_);
                    std::cout << "Elapsed: " << elapsed << "s Iterations: " << std::scientific << (double)iterSum << " g. poses " << poseCount.load() << std::endl; // "\r";
                }
                else
                {
                    std::stringstream t;
                    t << "Elapsed: " << elapsed << " ";
                    for (size_t i = 0; i < THREADS_USED; i++)
                    {
                        t << "T" << i << " :" << iterCount[i].load() << " ";
                    }
                    {
                        std::lock_guard<std::mutex> lock(mtx_cout_);
                        std::cout << t.str() << std::endl;
                    }
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

    std::stringstream t;
    t << "Tried " << std::scientific << (double)std::accumulate(iterCount.begin(), iterCount.end(), 0) << " poses. " << std::endl;
    for (size_t i = 0; i < THREADS_USED; i++)
    {
        t << "T" << i << " :" << iterCount[i].load() << " ";
    }
    std::cout << t.str() << std::endl;
    // std::cout << "Tried " << std::scientific << (double)std::accumulate(iterCount.begin(), iterCount.end(), 0) << " poses!" << std::endl;
    std::cout << "Found " << std::scientific << (double)poseCount.load() << " good poses!" << std::endl;

    return 0;
}
