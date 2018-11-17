#include <iostream>
#include <fstream>
#include <thread>
#include <algorithm>
#include <numeric>
#include <limits>
#include <cmath>

#include <Data/CameraMatrix.hpp>
#include <Modules/NaoProvider.h>
#include <Modules/Configuration/Configuration.h>
#include <Modules/Configuration/UnixSocketConfig.hpp>
#include <Modules/Poses.h>

#include <Tools/Storage/Image.hpp>
#include <Tools/Storage/Image422.hpp>
#include <Tools/Kinematics/KinematicMatrix.h>
#include <Hardware/RobotInterface.hpp>

#include "constants.hpp"
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
static const int SENSOR_COUNT = 4;

typedef std::array<float, JOINTS::JOINT::JOINTS_MAX> JointWeights;
typedef std::array<float, SENSOR_COUNT> SensorWeights;

typedef std::pair<int, std::string> SensMagAndPoseId; // Sensor Name should be maintained externally.
typedef std::pair<double, std::string> PoseCost;      // Pose cost and ID

static const size_t maxPeakElemsPerList = 10E15;
static const size_t maxPeakElemsPerListSortTrigger = 10E18;
static const size_t finalPoseFilteringLimit = 1000;

void sortDescAndTrimPoseCostVector(std::vector<PoseCost> &vec, const size_t &elemLimit)
{
    // sort in asc. order
    std::sort(vec.begin(),
              vec.end());
    // erase the excessive elements.
    if (vec.size() > elemLimit)
    {
        vec.erase(vec.begin() + elemLimit, vec.end());
    }
}

/**
 * Cost function for ranking a given pose for a given sensor.
 * Factors:
 * 1. Sensitivity magnitude
 * 2. Mutual orthogonality
 * Weights:
 * 1. w_sensor_mag - can be used to prioritize a given sensor's magnitudes
 * 2. w_joint - can be used to prioritize a given joint (esp. useful when general optimization only favor some joints)
 * 3. w_sensor_ortho - Orthogonality weight for a given sensor.
 * 
 * w_sensor_mag * w_joint and w_sensor_ortho * w_joint are used for magnitude and orthogonality weighting respectively.
 */
template <typename T>
double poseFilterCostFunc(const PoseSensitivity<T> &p, const float &wSensorMag,
                          const JointWeights &wJoints, const float &wOrthoGeneric,
                          const float &observableJointCountWeight)
{
    double accum = observableJointCountWeight * p.getObservableCount();
    if (std::isnan(accum))
    {
        std::lock_guard<std::mutex> lock(utils::mtx_cout_);
        std::cout << p.getId() << " is NAN 0 stage" << std::endl;
        return std::numeric_limits<double>::max();
    }
    for (int j = 0; j < static_cast<int>(JOINTS::JOINT::JOINTS_MAX); j++)
    {
        const JOINTS::JOINT joint = static_cast<JOINTS::JOINT>(j);
        if (!p.isJointObservable(joint))
        {
            continue;
        }
        const float wMag = wSensorMag * wJoints[j];
        const float wOrtho = wOrthoGeneric * wJoints[j];
        T val;
        bool obs;
        p.getSensitivity(joint, val, obs);
        // stage 1
        accum += wMag * val.norm();
        if (std::isnan(val.norm()) || std::isnan(accum))
        {
            std::lock_guard<std::mutex> lock(utils::mtx_cout_);
            std::cout << p.getId() << " is NAN 1st stage joint: " << j << " val " << val.norm() << std::endl;
            continue;
        }

        double v1Norm = val.norm();
        for (int k = 0; k < static_cast<int>(JOINTS::JOINT::JOINTS_MAX); k++)
        {
            const JOINTS::JOINT innerJoint = static_cast<JOINTS::JOINT>(k);
            // skip own-joint and unobservable joints..
            if (j == k || !p.isJointObservable(innerJoint))
            {
                continue;
            }
            T val2;
            bool obs2;
            p.getSensitivity(innerJoint, val2, obs2);

            double v2Norm = val2.norm();
            // second stage
            double dot = std::fabs((double)val.dot(val2) / (v1Norm * v2Norm));
            // We trust in the dot product :P
            if (dot > 1.0)
            {
                dot = 1.0;
            }
            else if (dot < -1.0)
            {
                dot = -1.0;
            }
            accum += wOrtho * std::fabs(std::acos(dot) / TO_RAD);
            if (dot > (double)1.0 || dot < (double)-1.0 || std::isnan(dot) || std::isnan(accum))
            {
                std::lock_guard<std::mutex> lock(utils::mtx_cout_);
                std::cout << p.getId() << " is NAN 2nd stage [domain err] " << j << " " << k << " dot " << dot << " aco " << std::acos(dot) << " wOrtho " << wOrtho << " fabs " << std::fabs(std::acos(dot) / TO_RAD) << std::endl;
                continue;
            }
        }
    }
    return -accum;
}

/**
 * This runs the cost function :)
 * @param poseCosts list of all cost minimas (local minimas).
 */
void poseFilterFunc(std::istream &inputStream,
                    // std::ostream &outputStream,
                    std::vector<PoseCost> &poseCosts,
                    const SensorWeights sensorMagnitudeWeights,
                    const JointWeights jointWeights,
                    const float orthogonalityWeight,
                    const float observableJointCountWeight,
                    std::atomic<size_t> &iterations)
{
    bool direction = false;
    PoseCost previousCostVal(-10, "-100");

    // Per pose, multiple sensors are there,..
    std::string curPoseId = "-100";
    // if at least one sensor had observations at that pose. If not, drop that pose from this level.
    // bool anySensorObserved = false;

    PoseSensitivity<Vector3f> p;
    size_t sensorAsIndex;

    double poseCostAccum = 0;

    while (utils::getNextDataEntry<PoseSensitivity<Vector3f>>(inputStream, p))
    {
        iterations++;
        // if nothing to observe, skip :P
        // TODO penalize poses that aren't much observable by sensors?
        if (p.getObservableCount() <= 0)
        {
            continue;
        }
        std::string tempId = p.getId();
        // Things don't match up = new pose loaded.
        if (curPoseId.compare(tempId))
        {
            // current pose's total cost is lesser than previous pose's total cost
            bool curDir = (poseCostAccum < previousCostVal.first);
            // falling edge - at a peak
            if (!curDir && direction)
            {
                poseCosts.emplace_back(poseCostAccum, curPoseId);
            }
//            if (poseCosts.size() > maxPeakElemsPerListSortTrigger)
//            {
//                sortDescAndTrimPoseCostVector(poseCosts, maxPeakElemsPerList);
//            }
            previousCostVal.first = poseCostAccum;
            previousCostVal.second = curPoseId;
            poseCostAccum = 0; // reset the accum
            curPoseId = tempId;
            direction = curDir;
            // std::cout << "new sensor" << std::endl;
        }
        sensorAsIndex = static_cast<int>(p.getSensorName());

        poseCostAccum += poseFilterCostFunc<Vector3f>(
            p,
            sensorMagnitudeWeights[sensorAsIndex],
            jointWeights,
            orthogonalityWeight,
            observableJointCountWeight);
    }

    // std::cout << "finishing" << std::endl;

    // sort ASC by cost and trim
//    sortDescAndTrimPoseCostVector(poseCosts, maxPeakElemsPerList);
}

int main(int argc, char **argv)
{
    std::string inFileName((argc > 1 ? argv[1] : "out"));
    std::string favouredJoint((argc > 2 ? argv[2] : "-1"));
    std::string confRoot((argc > 3 ? argv[3] : "../../nao/home/"));

    int jointNum = std::stoi(favouredJoint);
    if (jointNum < 0 || jointNum >= static_cast<int>(JOINTS::JOINT::JOINTS_MAX))
    {
        jointNum = -1;
    }

    const std::string outCostsFileName(inFileName + "_" + constants::FilteredPoseCostsFileName + "_" + (jointNum >= 0 ? "j" + std::to_string(jointNum) : "generic"));
    const std::string outFilteredPosesFileName(inFileName + "_" + constants::FilteredPosesFileName + "_" + (jointNum >= 0 ? "j" + std::to_string(jointNum) : "generic"));

    TUHH tuhhInstance(confRoot);

    std::cout << "init" << std::endl;

    size_t usableThreads = 0;

    std::vector<std::fstream> inputPoseAndJointStreams;
    for (size_t i = 0; i < MAX_THREADS; i++)
    {
        std::string fileName = inFileName + "_" + constants::ExtractedSensitivitiesFileName + "_" + std::to_string(i) + ".txt";
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
    std::fstream outCostsFile = std::fstream((outCostsFileName + ".txt"), std::ios::out);
    std::fstream outFilteredPosesFile = std::fstream((outFilteredPosesFileName + ".txt"), std::ios::out);

    if (!outCostsFile.is_open() || !outFilteredPosesFile.is_open())
    {
        std::cerr << "output file creation failed. Aborting!!!" << std::endl;
        exit(1);
    }

    /// Start the real threading..
    {
        std::vector<std::thread> threadList(usableThreads);
        std::vector<std::atomic<size_t>> iterCount(usableThreads);

        // threads -> sensors -> joints -> peakList (PerJointPerSensorPerThread)
        std::vector<std::vector<PoseCost>> poseCostListList(usableThreads);
        SensorWeights sensorMagnitudeWeights;
        sensorMagnitudeWeights.fill(2);

        // TODO change this as needed.
        JointWeights jointWeights;
        jointWeights.fill(1);
        if (jointNum > 0 && jointNum < static_cast<int>(JOINTS::JOINT::JOINTS_MAX))
        {
            jointWeights[jointNum] = 4;
        }

        const float wOrthoGeneric = 3;
        const float observableJointCountWeight = 1;

        for (unsigned int i = 0; i < usableThreads; i++)
        {

            // void poseFilterFunc(std::istream &inputStream,
            //                     // std::ostream &outputStream,
            //                     std::vector<PoseCost> &poseCosts,
            //                     const SensorWeights sensorMagnitudeWeights,
            //                     const JointWeights jointWeights,
            //                     const float orthogonalityWeight,
            //                     std::atomic<size_t> &iterations, bool lastPortion)

            threadList[i] = std::thread(poseFilterFunc, std::ref(inputPoseAndJointStreams[i]),
                                        // std::ref(outputFileList[i]),
                                        std::ref(poseCostListList[i]),
                                        sensorMagnitudeWeights,
                                        jointWeights,
                                        wOrthoGeneric,
                                        observableJointCountWeight,
                                        std::ref(iterCount[i]));
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
        // first join all
        std::cout<< poseCostListList[0].size()<< " ";
        for (size_t t = 1; t < usableThreads; t++) //per thread
        {
            std::cout<< poseCostListList[t].size() <<" ";
            poseCostListList[0].insert(poseCostListList[0].end(),
                                       std::make_move_iterator(poseCostListList[t].begin()),
                                       std::make_move_iterator(poseCostListList[t].end()));
        }
        std::cout<< std::endl;
        std::vector<PoseCost> &sortedPoseCostVec = poseCostListList[0];

        std::cout<<"total filtered pose count " << sortedPoseCostVec.size() <<std::endl;
// sort the results by ID. Will be expensive -_- But will save time in lookup.
//        std::sort(sortedPoseCostVec.begin(), sortedPoseCostVec.end(), [](PoseCost & a, PoseCost & b) {
//            return a.second < b.second;
//        });

//        /// Lookup the poses and get to memory.
//        // actual poses here..
        std::vector<poseAndRawAngleT> poseList(sortedPoseCostVec.size());
//        std::map<std::string, poseAndRawAngleT>::iterator poseMapIter;

        std::fstream genPoseListFile;

        size_t outFileNum = 0;
        std::cout<<"opening "<< outFileNum << std::endl;
        genPoseListFile.open((inFileName + "_GeneratedPoses_" + std::to_string(outFileNum) + ".txt"), std::ios::in);

//        size_t triedLines = 0;

        for (size_t idx = 0; idx < sortedPoseCostVec.size(); idx++) //per thread
        {
            auto & elem = sortedPoseCostVec[idx];

            std::string curString = "";
            std::string dummy = "";
            std::string id = "";
            bool elementFound = false;

            poseAndRawAngleT tempPose;

            while(std::getline(genPoseListFile, curString)){
//                triedLines ++;

                std::stringstream curStream(curString);

                curStream >> dummy >> dummy >> id;
                if(elem.second.compare(id) == 0){
                    elementFound = true;
                    curStream.seekg (0, curStream.beg);
                    curStream >> poseList[idx];
                    break;
                }
            }

            if(!genPoseListFile.good()){
                genPoseListFile.close();
                outFileNum++;
                if(outFileNum < usableThreads){
                    if(!elementFound){
                        idx--; // go back one index
                    }
                    std::cout<<"opening "<< outFileNum << std::endl;
                    genPoseListFile.open((inFileName + "_GeneratedPoses_" + std::to_string(outFileNum) + ".txt"), std::ios::in);
                }else{
                    if(idx + 1 < sortedPoseCostVec.size()){
                        std::cout<<"No more files to read " << idx << " " << sortedPoseCostVec.size() <<std::endl;
                    }
                    break;
                }
            }else{
                if(!elementFound){
                    std::cout << "element not found"<< elem.second << std::endl;
                }
            }
        }
//        // close the file
        genPoseListFile.close();

//        /// Sort filtered poses in cost order and commit cost and pose seperately.
//        // now sort the filtered poses in
        std::sort(sortedPoseCostVec.begin(),
                  sortedPoseCostVec.end());

        if(poseList.size() != sortedPoseCostVec.size()){
            std::cerr << "PoseMap and PoseCostVec doesnt match in size" << poseList.size() << " " << sortedPoseCostVec.size() << std::endl;
            return 1;
        }

        const size_t trimmedLen = std::min(finalPoseFilteringLimit, sortedPoseCostVec.size());

        for (size_t i = 0; i < trimmedLen; i++)
        {
            try {
                outFilteredPosesFile << poseList[i] << std::endl;
                outCostsFile << "poseCost " << sortedPoseCostVec[i].second << " " << sortedPoseCostVec[i].first << std::endl;
            } catch (std::exception e) {
                std::cerr << e.what() << " " << sortedPoseCostVec[i].second << std::endl;
                return 1;
            }

        }

        // outputFileList[t].close();
        outCostsFile.close();
        outFilteredPosesFile.close();
#endif
    }
    return 0;
}
