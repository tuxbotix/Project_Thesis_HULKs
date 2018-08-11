#include <iostream>
#include <fstream>
#include <vector>
#include <thread>
#include <mutex>
#include <algorithm>
#include <functional>
#include <sstream>
#include <atomic>
#include <chrono>
#include <cmath>

#define WRITE_PARALLEL 1
#include "utils.hpp"

static std::mutex mtx_;
static std::mutex mtx_poseList_;

typedef float dataT;
typedef int jointT;
typedef std::vector<dataT> poseT;
typedef std::vector<poseT> poseListT;

const jointT JOINT_COUNT = 10;
// const int PRINT_INTERVAL

dataT global_min = 0;
dataT global_max = 4;

int iterCount = 0;

const unsigned int MAX_THREADS = std::thread::hardware_concurrency(); // - 1;
//2000MB(approx)  total buffer usage = BUFFER_SIZE *sizeof(int) * JOINT_COUNT * MAX_THREADS
const long totalBufferSizeInBytes = 3E9;
const int BUFFER_SIZE = totalBufferSizeInBytes / (sizeof(dataT) * JOINT_COUNT * MAX_THREADS);

poseT joints(JOINT_COUNT);

inline bool finalProcess(poseT &vec)
{
    std::lock_guard<std::mutex> lock(mtx_);
    iterCount++;

    return true;
}

inline void recursiveIter(const jointT &joint, dataT start, dataT end, poseListT &poseList, poseT &accum, std::ostream &outStream, const bool &inclusiveMax);
inline void recursiveIter(const jointT &joint, dataT start, dataT end, poseListT &poseList, poseT &accum, std::ostream &outStream, const bool &inclusiveMax)
{

    for (dataT i = start; i < end || (inclusiveMax ? i == end : false); i++)
    {
#if DEBUG_IN_THREAD
        if (joint == 0)
        {
            std::lock_guard<std::mutex> lock(mtx_cout_);
            std::cout << "iter " << inclusiveMax<< " "<<start<<" "<<end << std::endl;
        }
#endif
        accum[joint] = i;
        int nextJoint = joint + 1;
        // if at end of chain, do this.
        if (nextJoint == JOINT_COUNT)
        {
            if (finalProcess(accum))
            {
                // std::lock_guard<std::mutex> lock(mtx_poseList_);
                poseList.push_back(accum);
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
            }
        }
        else
        {
            recursiveIter(nextJoint, global_min, global_max, poseList, accum, outStream, true);
        }
    }
}

int main(int argc, char **argv)
{
    std::string outFileName((argc > 1 ? argv[1] : "out"));

    std::cout << "# init" << std::endl;
    jointT initialJoint = 0;

    const dataT increment = 1;

    const int count = std::ceil(abs(global_max - global_min) / (dataT)increment);
    const dataT splitVal = (dataT)count * increment / (dataT)MAX_THREADS;

    const size_t THREADS_USED = (splitVal > 0) ? MAX_THREADS : 1;

    std::vector<poseListT> poseListList(THREADS_USED);
    poseListT accumList(THREADS_USED);

    std::cout << "# Will launch " << THREADS_USED << " threads." << std::endl;
    for (auto &i : accumList)
    {
        i = poseT(JOINT_COUNT);
    }
    for (auto &i : poseListList)
    {
        i.reserve(BUFFER_SIZE); // = poseListT(BUFFER_SIZE);
    }

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
            dataT start = (global_min + i * splitVal);
            const bool lastIter = (i + 1 == THREADS_USED);
            dataT end = lastIter ? global_max : (global_min + (i + 1) * splitVal);
            std::cout << "Start, end, lastIter " << start << " " << end << " " << lastIter << std::endl;
            threadList[i] = std::thread(recursiveIter, 0, start, end, std::ref(poseListList[i]),
                                        std::ref(accumList[i]), std::ref(outputFileList[i]), lastIter);
        }
        for (auto &t : threadList)
        {
            if (t.joinable())
            {
                t.join();
            }
        }
        std::cout << "flushing all " << std::endl;
        for (int i = 0; i < THREADS_USED; i++)
        {
            commitToStream<dataT>(poseListList[i], outputFileList[i]);
            outputFileList[i].close();
        }
    }

    std::cout << "# Poses generated" << std::endl;
    std::cout << "# iters: " << iterCount << std::endl;
}
