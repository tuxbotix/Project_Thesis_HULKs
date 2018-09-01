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
#define ENABLE_PROGRESS 0

#include "utils.hpp"

const unsigned int MAX_THREADS = std::thread::hardware_concurrency();
static const int SENSOR_COUNT = 4;

static const size_t maxPeakElemsPerList = 100;
static const size_t maxPeakElemsPerListSortTrigger = 150;

typedef std::pair<int, std::string> SensMagAndPoseId; // Sensor Name should be maintained externally.

typedef int16_t dataOutType;

void sortDescAndTrimSensPeakVector(std::vector<SensMagAndPoseId> &vec, const size_t &elemLimit)
{
    // sort in desc. order
    std::sort(vec.begin(),
              vec.end(), [](const SensMagAndPoseId &l, const SensMagAndPoseId &r) {
                  return l.first > r.first;
              });
    // erase the excessive elements.
    if (vec.size() > elemLimit)
    {
        vec.erase(vec.begin() + elemLimit, vec.end());
    }
}
/**
 * This simply get the highest sensitive poses per joint per sensor.
 * No effort is made to cluster, nearest neighbour or etc..
 */
void poseFilterFunc(std::istream &inputStream,
                    // std::ostream &outputStream,
                    std::array<std::vector<std::vector<SensMagAndPoseId>>, SENSOR_COUNT> &sensPeaksPerSensorPerJoint,
                    std::array<std::vector<SensMagAndPoseId>, SENSOR_COUNT> &maxSensMagnitudes,
                    std::atomic<size_t> &iterations, bool lastPortion)
{
    for (auto &s : sensPeaksPerSensorPerJoint)
    {
        s = std::vector<std::vector<SensMagAndPoseId>>(JOINTS::JOINT::JOINTS_MAX, std::vector<SensMagAndPoseId>());
    }
    std::vector<bool> direction[SENSOR_COUNT]; // just in case :P
    for (auto &i : direction)
    {
        i = std::vector<bool>(JOINTS::JOINT::JOINTS_MAX, false);
    }
    std::vector<SensMagAndPoseId> previousSenseVals[SENSOR_COUNT];
    for (auto &i : previousSenseVals)
    {
        i = std::vector<SensMagAndPoseId>(JOINTS::JOINT::JOINTS_MAX, SensMagAndPoseId(-32000, "-100"));
    }
    for (auto &i : maxSensMagnitudes)
    {
        i = std::vector<SensMagAndPoseId>(JOINTS::JOINT::JOINTS_MAX, SensMagAndPoseId(-32000, "-100"));
    }

    // Per pose, multiple sensors are there,..
    std::string curPoseId = "-100";
    // if at least one sensor had observations at that pose. If not, drop that pose from this level.
    // bool anySensorObserved = false;

    Vector3f val;
    bool observable;

    PoseSensitivity<Vector3f> p;
    int sensorAsId;
    JOINTS::JOINT joint;

    while (utils::getNextDataEntry<PoseSensitivity<Vector3f>>(inputStream, p))
    {
        iterations++;
        // if nothing to observe, skip :P
        if (p.getObservableCount() <= 0)
        {
            continue;
        }
        curPoseId = p.getId();

        sensorAsId = static_cast<int>(p.getSensorName());
        bool anyPeaksForThisPose = false;
        for (int j = 0; j < static_cast<int>(JOINTS::JOINT::JOINTS_MAX); j++)
        {
            joint = static_cast<JOINTS::JOINT>(j);
            p.getSensitivity(joint, val, observable);
            float sensMagnitude = val.norm();
            if (!observable || sensMagnitude < __FLT_EPSILON__)
            {
                continue;
            }

            SensMagAndPoseId prevVal = previousSenseVals[sensorAsId][j];
            bool curDir = prevVal.first < sensMagnitude;
            // falling edge = previous value is the local peak
            if (!curDir && direction[sensorAsId][j])
            {
                anyPeaksForThisPose = true;
                sensPeaksPerSensorPerJoint[sensorAsId][j].push_back(prevVal);
                size_t peakListSize = sensPeaksPerSensorPerJoint[sensorAsId][j].size();
                if (peakListSize > maxPeakElemsPerListSortTrigger)
                {
                    sortDescAndTrimSensPeakVector(sensPeaksPerSensorPerJoint[sensorAsId][j], maxPeakElemsPerList);
                }
            }
            // increasing
            if (curDir && maxSensMagnitudes[sensorAsId][j].first < sensMagnitude)
            {
                // update glocal [global for this section of data]
                maxSensMagnitudes[sensorAsId][j].first = sensMagnitude;
                maxSensMagnitudes[sensorAsId][j].second = curPoseId;
            }
            direction[sensorAsId][j] = curDir;
            // update history
            previousSenseVals[sensorAsId][j].first = sensMagnitude;
            previousSenseVals[sensorAsId][j].second = curPoseId;
        }
    }

    std::cout << "finishing" << std::endl;
    // Now the list has ended, and if the magnitude was rising, we'll add this as a peak.
    for (int s = 0; s < SENSOR_COUNT; s++)
    {
        for (int j = 0; j < static_cast<int>(JOINTS::JOINT::JOINTS_MAX); j++)
        {
            if (lastPortion && direction[s][j])
            {
                sensPeaksPerSensorPerJoint[s][j].push_back(previousSenseVals[s][j]);
            }
            // sort DESC by magnitude and trim
            sortDescAndTrimSensPeakVector(sensPeaksPerSensorPerJoint[sensorAsId][j], maxPeakElemsPerList);
            // sort by ID ASC
            std::sort(sensPeaksPerSensorPerJoint[sensorAsId][j].begin(),
                      sensPeaksPerSensorPerJoint[sensorAsId][j].end(),
                      [](const SensMagAndPoseId &l, const SensMagAndPoseId &r) {
                          return l.second < r.second;
                      });
        }
    }
}

int main(int argc, char **argv)
{
    std::string inFileName((argc > 1 ? argv[1] : "out"));
    std::string confRoot((argc > 2 ? argv[2] : "../../nao/home/"));

    const std::string outFileName(inFileName + "_PoseFilterOut");

    TUHH tuhhInstance(confRoot);

    std::cout << "init" << std::endl;

    size_t usableThreads = 0;

    std::vector<std::fstream> inputPoseAndJointStreams;
    for (size_t i = 0; i < MAX_THREADS; i++)
    {
        std::string fileName = inFileName + "_sensOut_" + std::to_string(i) + ".txt";
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
    std::cout << "make outfiles. Usable threads -> " << usableThreads << std::endl;
    std::fstream outputFile = std::fstream((outFileName + ".txt"), std::ios::out);
    if (!outputFile.is_open())
    {
        std::cerr << "output file creation failed. Aborting!!!" << std::endl;
        exit(1);
    }

    /// Start the real threading..
    {
        std::vector<std::thread> threadList(usableThreads);
        std::vector<std::atomic<size_t>> iterCount(usableThreads);

        // threads -> sensors -> joints -> peakList (PerJointPerSensorPerThread)
        std::vector<std::array<std::vector<std::vector<SensMagAndPoseId>>, SENSOR_COUNT>> sensPeakPerSensorPerJointList(usableThreads);
        std::vector<std::array<std::vector<SensMagAndPoseId>, SENSOR_COUNT>> maxPeaksPerSensorPerJoint(usableThreads);

        for (unsigned int i = 0; i < usableThreads; i++)
        {

            //          void poseFilterFunc(ObservationSensitivity &obs, std::istream &inputStream, std::ostream &outputStream,
            //                     std::array<std::vector<std::vector<SensMagAndPoseId>>, SENSOR_COUNT> &sensPeaksPerSensorPerJoint,
            //                     std::array<std::vector<SensMagAndPoseId>, SENSOR_COUNT> &maxSensMagnitudes,
            //                     std::atomic<size_t> &iterations, bool lastPortion)
            threadList[i] = std::thread(poseFilterFunc, std::ref(inputPoseAndJointStreams[i]),
                                        // std::ref(outputFileList[i]),
                                        std::ref(sensPeakPerSensorPerJointList[i]),
                                        std::ref(maxPeaksPerSensorPerJoint[i]),
                                        std::ref(iterCount[i]), (i + 1 >= usableThreads));
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
        for (size_t t = 0; t < usableThreads; t++) //per thread
        {
            for (size_t s = 0; s < SENSOR_COUNT; s++) // per sensor
            {
                outputFile << "sensor " << s << std::endl;
                for (size_t j = 0; j < sensPeakPerSensorPerJointList[t][s].size(); j++) // per joint
                {
                    if (sensPeakPerSensorPerJointList[t][s][j].size() <= 0)
                    {
                        continue;
                    }
                    outputFile << "joint " << j << std::endl;
                    for (auto &sensPeak : sensPeakPerSensorPerJointList[t][s][j])
                    {
                        // pose ID and sens value
                        outputFile << sensPeak.second << " " << sensPeak.first << std::endl;
                    }
                    outputFile << "jointEnd " << j << std::endl;
                }
                outputFile << "sensorEnd " << s << std::endl;
            }
            // outputFileList[t].close();
            outputFile.close();
        }
#endif
    }

    return 0;
}
