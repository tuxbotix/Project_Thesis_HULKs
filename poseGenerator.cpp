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
// #include "ObservationSensitivityProvider.hpp"

#define DO_COMMIT 1
#define WRITE_PARALLEL 1
#define DEBUG_FILE_COMMIT 1
#define DEBUG_IN_THREAD 1
#define ENABLE_PROGRESS 1

#include "utils.hpp"

inline void doLog(const std::string &value)
{
    {
#if DEBUG_IN_THREAD
        std::lock_guard<std::mutex> lck(utils::mtx_cout_);
#endif
        std::cout << value << std::endl;
    }
}

const unsigned int MAX_THREADS = std::thread::hardware_concurrency();

//2000MB(approx)  total buffer usage = BUFFER_SIZE *sizeof(int) * JOINT_COUNT * MAX_THREADS
const size_t totalBufferSizeInBytes = 200E6;
const size_t BUFFER_SIZE = totalBufferSizeInBytes / (sizeof(dataT) * static_cast<size_t>(PARAMS::P_MAX) * MAX_THREADS);

std::atomic<size_t> poseCount(0);

/**
 * Called at end of each update of joint angle
 */
inline bool poseCallback(rawAnglesT &poseByAngles, const SUPPORT_FOOT &supFoot, const SupportPolygon &supportPoly)
{
    /// Where would the com be after setting these angles?
    KinematicMatrix com2torso = KinematicMatrix(Com::getCom(poseByAngles));

    KinematicMatrix lFoot2torso = ForwardKinematics::getLFoot(rawPoseT(&poseByAngles[JOINTS::L_HIP_YAW_PITCH], &poseByAngles[JOINTS::L_ANKLE_ROLL]));
    KinematicMatrix rFoot2torso = ForwardKinematics::getRFoot(rawPoseT(&poseByAngles[JOINTS::R_HIP_YAW_PITCH], &poseByAngles[JOINTS::R_ANKLE_ROLL]));

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
    minLim = -72;
    maxLim = 72;
    increment = 18;
    // minLim = -1;
    // maxLim = 1;
    // increment = 1;
    return;
}
inline void getHeadPitchLimits(const dataT &headYaw, dataT &minLim, dataT &maxLim, dataT &increment) // 19
{
    minLim = std::max(NaoProvider::minRangeHeadPitch(headYaw * TO_RAD), 0.0f) / TO_RAD;
    maxLim = NaoProvider::maxRangeHeadPitch(headYaw * TO_RAD) / TO_RAD;
    // minLim = 0;
    // maxLim = 2;
    increment = 1;
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
    else if (paramIndex == PARAMS::P_SUPPORT_FOOT)
    {
        std::lock_guard<std::mutex> lock(utils::mtx_cout_);
        std::cout << "SOMETHING IS WRONG, This method should not call head or support foot!!!" << paramIndex << std::endl; // "\r";
        return;
    }

    switch (paramIndex)
    {
    // torsoPosVector
    case PARAMS::P_TORSO_POS_X:
    case PARAMS::P_TORSO_POS_Y:
    {
        minLim = -0.1;
        maxLim = 0.1;
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
    {
        minLim = -12;
        maxLim = 12;
        increment = 3; // 2 deg increment - 5
        break;
    }
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
        break;
    }
    case PARAMS::P_O_FOOT_POS_Y:
    {
        if (supFoot == SUPPORT_FOOT::SF_RIGHT) // 5
        {
            minLim = 0.0;
            maxLim = 0.15;
        }
        else // 5
        {
            minLim = -0.15;
            maxLim = 0; //0.0;
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
            std::lock_guard<std::mutex> lock(utils::mtx_cout_);
            std::cout << "Default should not be reached.. method should not call head or support foot!!!" << paramIndex << std::endl; // "\r";
        }
        minLim = 0;
        maxLim = 0;
        increment = 0.1;
        break;
    }
    };
}

void jointIterFuncWithLim(const size_t torsoPosBegin, const size_t torsoPosEnd, const SUPPORT_FOOT supFoot,
                          const std::vector<HeadYawPitch> &headYawPitchList, const vector3ListT torsoPosList, const vector3ListT torsoRotList,
                          const vector3ListT OFootPosList,
                          const vector3ListT OFootRotList, poseAndRawAngleListT &poseList, std::ostream &poseOutStream,
                          const SupportPolygon &supportPoly, std::atomic<size_t> &iterCount, const std::string &id_prefix)
{
    const rawAnglesT defaultPose = Poses::getPose(Poses::READY);
    for (size_t iter = torsoPosBegin; iter < torsoPosEnd; ++iter)
    {
        Vector3<dataT> torsoPos(torsoPosList[iter]);
        for (auto torsoRot : torsoRotList)
        {
            for (auto OFootPos : OFootPosList)
            {
                /// Prefiltering.. Check for collisions and others here. :)
                /// First check, double foot..
                if (supFoot == SUPPORT_FOOT::SF_DOUBLE && OFootPos.z() > __FLT_EPSILON__)
                {
                    iterCount += (OFootRotList.size() * headYawPitchList.size());
                    continue;
                }
                for (auto OFootRot : OFootRotList)
                {
                    iterCount += headYawPitchList.size();
                    /// Prefiltering.. Check for collisions and others here. :)
                    /// First check, double foot..
                    if (supFoot == SUPPORT_FOOT::SF_DOUBLE && (OFootRot.x() > __FLT_EPSILON__ ||
                                                               OFootRot.x() > __FLT_EPSILON__))
                    {
                        continue;
                    }
                    // TODO insert more checks here

                    rawAnglesT jointAngles = defaultPose;
                    bool poseGenSuccess = NaoTorsoPose::getPose(jointAngles,
                                                                torsoPos,
                                                                torsoRot * TO_RAD,
                                                                OFootPos,
                                                                OFootRot * TO_RAD,
                                                                supFoot);
                    // if pose gen success and pose is statically stable
                    if (poseGenSuccess)
                    {
                        for (size_t iter = 0; iter < headYawPitchList.size(); iter++)
                        {
                            jointAngles[JOINTS::HEAD_YAW] = headYawPitchList[iter].yaw * TO_RAD;
                            jointAngles[JOINTS::HEAD_PITCH] = headYawPitchList[iter].pitch * TO_RAD;
                            if (poseCallback(jointAngles, supFoot, supportPoly))
                            {
                                poseCount++;
#if DO_COMMIT
                                poseList.emplace_back(poseT(id_prefix + std::to_string(poseCount), supFoot, headYawPitchList[iter], torsoPos, torsoRot,
                                                            OFootPos, OFootRot),
                                                      jointAngles);
                                // rawPoseList.push_back(jointAngles);
                                if (poseList.size() > BUFFER_SIZE)
                                {
                                    utils::commitToStream<NaoPoseAndRawAngles<dataT>>(poseList, poseOutStream);
                                }
                                // if (rawPoseList.size() > BUFFER_SIZE)
                                // {
                                //     utils::commitToStreamVec<rawPoseT>(rawPoseList, jointOutStream);
                                // }
#endif
                            }
                        }
                    }
                }
            }
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

    const std::string timestamp = utils::getMilliSecondsString();

    TUHH tuhhInstance(confRoot);
    // const std::string dateTimeString = getISOTimeString();
    /// Pose Gen
    std::cout << "# Init for pose generation" << std::endl;

    const rawPoseT readyPose(PARAMS::P_MAX);

    std::vector<HeadYawPitch> headYawPitchList;
    vector3ListT torsoPosList;
    vector3ListT torsoRotList;
    vector3ListT OtherFootPosList;
    vector3ListT OtherFootRotList;
    {
        size_t count = 0;
        dataT minLimit, maxLimit, increment;
        getHeadYawLimits(minLimit, maxLimit, increment);
        for (dataT yaw = minLimit; yaw <= maxLimit; yaw += increment)
        {
            if (std::abs(yaw) < increment)
            {
                yaw = 0;
            }
            dataT minPitch, maxPitch, pitchIncr;
            getHeadPitchLimits(yaw, minPitch, maxPitch, pitchIncr);
            for (dataT pitch = minPitch; pitch <= maxPitch; pitch += pitchIncr)
            {
                if (std::abs(pitch) < pitchIncr)
                {
                    pitch = 0;
                }
                // count++;
                headYawPitchList.emplace_back(yaw, pitch);
            }
        }
        std::cout << count << std::endl;
    }
    size_t maxIterCount = headYawPitchList.size();
    {

        Vector3<dataT> minLimit, maxLimit, increment;
        for (int i = static_cast<int>(paramNameT::P_TORSO_POS_X); i < static_cast<int>(paramNameT::P_MAX); i += 3)
        {
            getLimits(static_cast<paramNameT>(i), supportFoot, minLimit.x(), maxLimit.x(), increment.x());
            getLimits(static_cast<paramNameT>(i + 1), supportFoot, minLimit.y(), maxLimit.y(), increment.y());
            getLimits(static_cast<paramNameT>(i + 2), supportFoot, minLimit.z(), maxLimit.z(), increment.z());
            switch (i)
            {
            case static_cast<int>(paramNameT::P_TORSO_POS_X):
                torsoPosList = PoseUtils::getVector3List<dataT>(minLimit, maxLimit, increment);
                maxIterCount *= torsoPosList.size();
                break;
            case static_cast<int>(paramNameT::P_TORSO_ROT_X):
                torsoRotList = PoseUtils::getVector3List<dataT>(minLimit, maxLimit, increment);
                maxIterCount *= torsoRotList.size();
                break;
            case static_cast<int>(paramNameT::P_O_FOOT_POS_X):
                OtherFootPosList = PoseUtils::getVector3List<dataT>(minLimit, maxLimit, increment);
                maxIterCount *= OtherFootPosList.size();
                break;
            case static_cast<int>(paramNameT::P_O_FOOT_ROT_X):
                OtherFootRotList = PoseUtils::getVector3List<dataT>(minLimit, maxLimit, increment);
                maxIterCount *= OtherFootRotList.size();
                break;
            default:
                std::cout << "something is wrong" << std::endl;
            }
        }
        std::cout << "Max iters -> " << std::scientific << (double)maxIterCount << std::endl;
    }
    /// Start threading work.
    std::cout << "Start Threading" << std::endl;
    const unsigned int count = torsoPosList.size();

    const dataT splitVal = count >= MAX_THREADS ? (std::floor((dataT)count / (dataT)MAX_THREADS)) : 1;
    std::cout << "1st level count " << std::defaultfloat << count << " " << splitVal << std::endl;
    const size_t THREADS_USED = (splitVal > 1) ? MAX_THREADS : count;

    /// PoseList vector and accum(pose) Vector
    std::vector<poseAndRawAngleListT> poseAndAnglesListList(THREADS_USED);
    // std::vector<rawPoseListT> jointListList(THREADS_USED);

    std::vector<std::atomic<size_t>> iterCount(THREADS_USED);
    /// Start the real threading..
    {
        std::vector<std::thread> threadList(THREADS_USED);
        std::vector<std::fstream> poseAndAnglesFiles(THREADS_USED);
        // std::vector<std::fstream> rawJointFiles(THREADS_USED);
        std::vector<SupportPolygon> supportPolyList(THREADS_USED);
        // bool resumeFlags[THREADS_USED];
        for (unsigned int i = 0; i < THREADS_USED; i++)
        {
            poseAndAnglesFiles[i] = std::fstream((outFileName + "_" + std::to_string(i) + ".txt"), std::ios::out);
            // poseAndAnglesFiles[i] = std::fstream((outFileName + "_joints_" + std::to_string(i) + ".txt"), std::ios::out);

            if (!poseAndAnglesFiles[i].is_open())
            {
                std::cerr << "output file creation failed. Aborting!!!" << std::endl;
                break;
            }
            const bool lastIter = (i + 1 == THREADS_USED);

            dataT start = (0 + ((dataT)i * splitVal));
            dataT end = lastIter ? count : (0 + (i + 1) * splitVal);

            std::cout << "Start, end, lastIter " << start << " " << end << " " << lastIter << std::endl;

            // void jointIterFuncWithLim(const size_t torsoPosBegin, const size_t torsoPosEnd, const SUPPORT_FOOT supFoot,
            //                           const std::vector<HeadYawPitch> &headYawPitchList, const vector3ListT torsoPosList, const vector3ListT torsoRotList,
            //                           const vector3ListT OFootPosList,
            //                           const vector3ListT OFootRotList, poseListT &poseList, std::ostream &poseOutStream,
            //                           const SupportPolygon &supportPoly, std::atomic<size_t> &iterCount)
            threadList[i] = std::thread(jointIterFuncWithLim, start, end, supportFoot,
                                        std::ref(headYawPitchList), std::ref(torsoPosList), std::ref(torsoRotList),
                                        std::ref(OtherFootPosList),
                                        std::ref(OtherFootRotList), std::ref(poseAndAnglesListList[i]), std::ref(poseAndAnglesFiles[i]),
                                        std::ref(supportPolyList[i]), std::ref(iterCount[i]), std::string(timestamp + std::to_string(i)));
        }
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
                    std::cout << "Elapsed: " << elapsed << "s Iterations: " << (iterSum / maxIterCount)
                              << "% g. poses " << (long double)poseCount.load() << std::endl; // "\r";
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
                        std::lock_guard<std::mutex> lock(utils::mtx_cout_);
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
            utils::commitToStream<NaoPoseAndRawAngles<dataT>>(poseAndAnglesListList[i], poseAndAnglesFiles[i]);
            poseAndAnglesFiles[i].close();
        }
#endif
    }

    std::cout << "Tried " << std::scientific << (long double)std::accumulate(iterCount.begin(), iterCount.end(), (size_t)0) << " poses!" << std::endl;
    std::cout << "Found " << std::scientific << (long double)poseCount.load() << " good poses!" << std::endl;

    return 0;
}
