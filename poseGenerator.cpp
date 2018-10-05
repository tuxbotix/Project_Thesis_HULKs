#include <iostream>
#include <fstream>
#include <thread>
#include <algorithm>
#include <numeric>
#include <cstdio>

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
#include "MiniConfigHandle.hpp"
#include "NaoPoseInfo.hpp"
#include "NaoStability.hpp"
#include "NaoTorsoPose.hpp"
// #include "NaoCollision.hpp"
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

class MinMaxInc
{
  public:
    Vector2f minHeadYawPitch;
    Vector2f maxHeadYawPitch;
    Vector2f incHeadYawPitch;
    std::array<Vector3f, 4> min;
    std::array<Vector3f, 4> max;
    std::array<Vector3f, 4> inc;

    inline void getHeadYawLimits(dataT &minLim, dataT &maxLim, dataT &increment) const // 8
    {
        // Instead of the full range, i'll limit..
        minLim = minHeadYawPitch[0];
        maxLim = maxHeadYawPitch[0];
        increment = incHeadYawPitch[0];
    }
    inline void getHeadPitchLimits(const dataT &headYaw, dataT &minLim, dataT &maxLim, dataT &increment) const // 19
    {
        minLim = std::max(NaoProvider::minRangeHeadPitch(headYaw * TO_RAD), 0.0f) / TO_RAD;
        maxLim = NaoProvider::maxRangeHeadPitch(headYaw * TO_RAD) / TO_RAD;
        increment = incHeadYawPitch[1];
    }

    inline void getLimits(const paramNameT &paramIndex, const SUPPORT_FOOT &supFoot, dataT &minLim, dataT &maxLim, dataT &increment) const
    {
        if (paramIndex >= static_cast<PARAMS>(PARAMS::P_MAX) || paramIndex < 0)
        {
            minLim = 0;
            maxLim = 0;
            increment = 0;
            std::lock_guard<std::mutex> lock(utils::mtx_cout_);
            std::cout << "param index out of bounds" << std::endl;
            return;
        }
        else if (paramIndex == PARAMS::P_SUPPORT_FOOT)
        {
            std::lock_guard<std::mutex> lock(utils::mtx_cout_);
            std::cout << "SOMETHING IS WRONG, This method should not call head or support foot!!!" << paramIndex << std::endl; // "\r";
            return;
        }
        paramNameT paramIdxCopy = static_cast<PARAMS>(paramIndex - PARAMS::P_TORSO_POS_X);

        // first array index
        uint8_t initialBlock = paramIdxCopy / 3;
        // second array index
        uint8_t subIdx = paramIdxCopy % 3;

        minLim = min[initialBlock][subIdx];
        maxLim = max[initialBlock][subIdx];
        increment = inc[initialBlock][subIdx];
    }

    static MinMaxInc populateMinMaxInc(const Uni::Value &val)
    {
        MinMaxInc minMaxIncObj;
        
        val["min_headYawPitch"] >> minMaxIncObj.minHeadYawPitch;
        val["min_torsoPos"] >> minMaxIncObj.min[0];
        val["min_torsoRot"] >> minMaxIncObj.min[1];
        val["min_oFootPos"] >> minMaxIncObj.min[2];
        val["min_oFootRot"] >> minMaxIncObj.min[3];

        val["max_headYawPitch"] >> minMaxIncObj.maxHeadYawPitch;
        val["max_torsoPos"] >> minMaxIncObj.max[0];
        val["max_torsoRot"] >> minMaxIncObj.max[1];
        val["max_oFootPos"] >> minMaxIncObj.max[2];
        val["max_oFootRot"] >> minMaxIncObj.max[3];

        val["inc_headYawPitch"] >> minMaxIncObj.incHeadYawPitch;
        val["inc_torsoPos"] >> minMaxIncObj.inc[0];
        val["inc_torsoRot"] >> minMaxIncObj.inc[1];
        val["inc_oFootPos"] >> minMaxIncObj.inc[2];
        val["inc_oFootRot"] >> minMaxIncObj.inc[3];

        return minMaxIncObj;
    }
};

const unsigned int MAX_THREADS = std::thread::hardware_concurrency();

//2000MB(approx)  total buffer usage = BUFFER_SIZE *sizeof(int) * JOINT_COUNT * MAX_THREADS
const size_t totalBufferSizeInBytes = 2E9;
const size_t BUFFER_SIZE = totalBufferSizeInBytes / (sizeof(dataT) * static_cast<size_t>(PARAMS::P_MAX) * MAX_THREADS);

std::atomic<size_t> poseCount(0);

/**
 * Called at end of each update of joint angle
 */
inline bool poseValidator(rawAnglesT &poseByAngles, const SUPPORT_FOOT &supFoot, float &com2CentroidDist, const SupportPolygon &supportPoly)
{
    /// Where would the com be after setting these angles?
    KinematicMatrix com2torso = KinematicMatrix(Com::getCom(poseByAngles));

    KinematicMatrix lFoot2torso = ForwardKinematics::getLFoot(rawPoseT(poseByAngles.begin() + JOINTS::L_HIP_YAW_PITCH,
                                                                       poseByAngles.begin() + JOINTS::L_ANKLE_ROLL + 1));
    KinematicMatrix rFoot2torso = ForwardKinematics::getRFoot(rawPoseT(poseByAngles.begin() + JOINTS::R_HIP_YAW_PITCH,
                                                                       poseByAngles.begin() + JOINTS::R_ANKLE_ROLL + 1));
    com2CentroidDist = 0;
    bool isStable = supportPoly.isComWithinSupport(lFoot2torso, rFoot2torso, com2torso, com2CentroidDist, supFoot);
    return isStable;
}

void jointIterFuncWithLim(const size_t torsoPosBegin, const size_t torsoPosEnd,
                          const SUPPORT_FOOT supFoot, const std::vector<HeadYawPitch> headYawPitchList,
                          const vector3ListT torsoPosList, const vector3ListT torsoRotList,
                          const vector3ListT OFootPosList, const vector3ListT OFootRotList,
                          poseAndRawAngleListT &poseList, std::ostream &poseOutStream,
                          const SupportPolygon &supportPoly /*, Collision::CollisionModel &collisionModel*/,
                          std::atomic<size_t> &iterCount, const std::string &id_prefix)
{
    const rawAnglesT defaultPose = Poses::getPose(Poses::READY);

    const auto begin = torsoPosList.begin() + torsoPosBegin;
    const auto end = torsoPosList.begin() + torsoPosEnd;
    for (auto torsoPos = begin; torsoPos != end; ++torsoPos)
    {
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
                    /// Prefiltering..
                    /// First check, double foot..
                    if (supFoot == SUPPORT_FOOT::SF_DOUBLE && (OFootRot.x() > __FLT_EPSILON__ ||
                                                               OFootRot.x() > __FLT_EPSILON__))
                    {
                        continue;
                    }

                    rawAnglesT jointAngles = defaultPose;
                    bool poseGenSuccess = NaoTorsoPose::getPose(jointAngles,
                                                                *torsoPos,
                                                                torsoRot * TO_RAD,
                                                                OFootPos,
                                                                OFootRot * TO_RAD,
                                                                supFoot);
                    // if pose gen success and pose is statically stable
                    // and if the pose doesn't cause collisions
                    // if (poseGenSuccess && collisionModel.updateAndTest(jointAngles))
                    if (poseGenSuccess)
                    {
                        for (const auto &headYawPitch : headYawPitchList)
                        {
                            jointAngles[JOINTS::HEAD_YAW] = headYawPitch.yaw * TO_RAD;
                            jointAngles[JOINTS::HEAD_PITCH] = headYawPitch.pitch * TO_RAD;
                            float com2CentroidDist = 0;
                            if (poseValidator(jointAngles, supFoot, com2CentroidDist, supportPoly))
                            {
                                poseCount++;
#if DO_COMMIT
                                poseList.emplace_back(poseT(id_prefix + std::to_string(poseCount), supFoot, com2CentroidDist, headYawPitch,
                                                            *torsoPos, torsoRot, OFootPos, OFootRot),
                                                      jointAngles);
                                if (poseList.size() > BUFFER_SIZE)
                                {
                                    utils::commitToStream<NaoPoseAndRawAngles<dataT>>(poseList, poseOutStream);
                                }
#endif
                            }
                        }
                    }
                }
            }
        }
    }
}

int main(int argc, char **argv)
{
    std::string outFileName((argc > 1 ? argv[1] : "out"));
    std::string supportFootName((argc > 2 ? argv[2] : "d"));
    std::string confRoot((argc > 3 ? argv[3] : "../../nao/home/"));

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
        supportFootName = "d";
    }

    const std::string timestamp = utils::getMilliSecondsString();

    TUHH tuhhInstance(confRoot);
    tuhhInstance.config_.mount("Projection", "Projection.json", ConfigurationType::HEAD);

    ///
    Uni::Value confValue;
    if (!MiniConfigHandle::mountFile("configuration/limits_" + supportFootName + ".json", confValue))
    {
        std::cout << "couldn't open the conf file" << std::endl;
        exit(1);
    }
    const MinMaxInc minMaxInobj = MinMaxInc::populateMinMaxInc(confValue);
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
        minMaxInobj.getHeadYawLimits(minLimit, maxLimit, increment);
        for (dataT yaw = minLimit; yaw <= maxLimit; yaw += increment)
        {
            if (std::abs(yaw) < increment)
            {
                yaw = 0;
            }
            dataT minPitch, maxPitch, pitchIncr;
            minMaxInobj.getHeadPitchLimits(yaw, minPitch, maxPitch, pitchIncr);
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
            minMaxInobj.getLimits(static_cast<paramNameT>(i), supportFoot, minLimit.x(), maxLimit.x(), increment.x());
            minMaxInobj.getLimits(static_cast<paramNameT>(i + 1), supportFoot, minLimit.y(), maxLimit.y(), increment.y());
            minMaxInobj.getLimits(static_cast<paramNameT>(i + 2), supportFoot, minLimit.z(), maxLimit.z(), increment.z());
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

    std::vector<std::atomic<size_t>> iterCount(THREADS_USED);
    /// Start the real threading..
    {
        std::vector<std::thread> threadList(THREADS_USED);
        std::vector<std::fstream> poseAndAnglesFiles(THREADS_USED);
        std::vector<SupportPolygon> supportPolyList(THREADS_USED);
        // std::vector<Collision::CollisionModel> collisionModelList(THREADS_USED);
        for (unsigned int i = 0; i < THREADS_USED; i++)
        {
            poseAndAnglesFiles[i] = std::fstream((outFileName + "_TempGeneratedPoses_" + std::to_string(i) + ".txt"), std::ios::out);

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
                                        headYawPitchList, torsoPosList, torsoRotList, OtherFootPosList, OtherFootRotList,
                                        std::ref(poseAndAnglesListList[i]), std::ref(poseAndAnglesFiles[i]),
                                        std::ref(supportPolyList[i]), /* std::ref(collisionModelList[i]), */ std::ref(iterCount[i]),
                                        std::string(timestamp + std::to_string(i)));
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
                    std::cout << "Elapsed: " << elapsed << "s Iterations: " << ((double)iterSum * 100 / maxIterCount)
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

#if DO_COMMIT
        std::cout << "flushing all " << std::endl;
        for (size_t i = 0; i < THREADS_USED; i++)
        {
            utils::commitToStream<NaoPoseAndRawAngles<dataT>>(poseAndAnglesListList[i], poseAndAnglesFiles[i]);
            poseAndAnglesFiles[i].close();
        }
        std::cout << "Redistributing data" << std::endl;
        // redistribute the written lines.
        {
            const size_t linesPerFile = poseCount / (size_t)THREADS_USED;
            size_t outFileNum = 0;
            std::fstream inStream;
            std::fstream outStream = std::fstream((outFileName + "_GeneratedPoses_" + std::to_string(outFileNum) + ".txt"), std::ios::out);
            std::cout << "Write to file: " << outFileNum << std::endl;
            size_t counter = 0;
            size_t totalCounter = 0;

            for (size_t i = 0; i < THREADS_USED; i++)
            {
                std::cout << "Read file: " << i << std::endl;
                inStream.open((outFileName + "_TempGeneratedPoses_" + std::to_string(i) + ".txt"), std::ios::in);
                if (!inStream)
                {
                    continue;
                }
                std::string s;
                while (std::getline(inStream, s))
                {
                    outStream << s << std::endl;
                    counter++;
                    totalCounter++;
                    if (counter > linesPerFile && (outFileNum + 1) < THREADS_USED) // if last file, keep appending to the same file.
                    {
                        outStream.close();
                        outStream.clear();
                        std::cout << "Finished writing to file: " << outFileNum << " written: " << counter << std::endl;
                        counter = 0;
                        outFileNum++;
                        std::cout << "Open write to file: " << outFileNum << std::endl;
                        outStream.open((outFileName + "_GeneratedPoses_" + std::to_string(outFileNum) + ".txt"), std::ios::out);
                    }
                }
                inStream.close();
                inStream.clear();
                // deleting the temp file.
                std::remove((outFileName + "_TempGeneratedPoses_" + std::to_string(i) + ".txt").c_str());
            }
            std::cout << "Total redistributed lines: " << totalCounter << std::endl;
        }

#endif
    }

    std::cout << "Tried " << std::scientific << (long double)std::accumulate(iterCount.begin(), iterCount.end(), (size_t)0) << " poses!" << std::endl;
    std::cout << "Found " << std::scientific << (long double)poseCount.load() << " good poses!" << std::endl;

    return 0;
}
