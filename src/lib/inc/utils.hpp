#pragma once

#include <cmath>
#include <vector>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <atomic>
#include <mutex>

namespace utils
{
#if !WRITE_PARALLEL
static std::atomic<bool> fileWriterReady(true);
#endif

#ifndef ENABLE_PROGRESS
#define ENABLE_PROGRESS 1
#endif

#ifndef DEBUG_FILE_COMMIT
#define DEBUG_FILE_COMMIT 1
#endif

#ifndef DEBUG_IN_THREAD
#define DEBUG_IN_THREAD 1
#endif

#ifndef USE_STRINGSTREAM
#define USE_STRINGSTREAM 0
#endif

#if DEBUG_IN_THREAD || DEBUG_FILE_COMMIT || ENABLE_PROGRESS
static std::mutex mtx_cout_;
#endif
template <typename T>
void commitToStream(std::vector<T> &poseList, std::ostream &outStream)
{
#if !WRITE_PARALLEL
    fileWriterReady = false;
#endif
#if DEBUG_FILE_COMMIT
    {
        std::lock_guard<std::mutex> lock(mtx_cout_);
        std::cout << "commit Start" << std::endl;
    }
#endif
#if USE_STRINGSTREAM
    std::stringstream buffer;
#endif
    for (const auto &p : poseList)
    {
#if USE_STRINGSTREAM
        buffer << p << "\n";
        if (buffer.tellg() > 4E6)
        { // write each 4MB :P
            std::cout << "buf write";
            outStream << buffer.str();
            buffer.str(std::string());
            buffer.clear();
        }
#else
        outStream << p << std::endl;
#endif
    }
#if USE_STRINGSTREAM
    outStream << buffer.str();
#endif
    poseList.clear();
#if DEBUG_FILE_COMMIT
    {
        std::lock_guard<std::mutex> lock(mtx_cout_);
        std::cout << "commit End" << std::endl;
    }
#endif
#if !WRITE_PARALLEL
    fileWriterReady = true;
#endif
    outStream.flush();
}

template <typename T>
void commitToStreamVec(std::vector<T> &poseList, std::ostream &outStream)
{
#if !WRITE_PARALLEL
    fileWriterReady = false;
#endif
#if DEBUG_FILE_COMMIT
    {
        std::lock_guard<std::mutex> lock(mtx_cout_);
        std::cout << "commit Start" << std::endl;
    }
#endif
#if USE_STRINGSTREAM
    std::stringstream buffer;
#endif
    for (const auto &i : poseList)
    {
        for (const auto &angle : i)
        {
#if USE_STRINGSTREAM
            buffer << angle << " ";
#else
            outStream << angle << " ";
#endif
        }
#if USE_STRINGSTREAM
        buffer << "\n";
        if (buffer.tellg() > 4E6)
        { // write each 4MB :P
            std::cout << "buf write";
            outStream << buffer.str();
            buffer.str(std::string());
            buffer.clear();
        }
#else
        outStream << "\n";
#endif
    }
#if USE_STRINGSTREAM
    outStream << buffer.str();
#endif
    poseList.clear();
#if DEBUG_FILE_COMMIT
    {
        std::lock_guard<std::mutex> lock(mtx_cout_);
        std::cout << "commit End" << std::endl;
    }
#endif
#if !WRITE_PARALLEL
    fileWriterReady = true;
#endif
    outStream.flush();
}

template <typename T>
std::vector<T> splitToNumbers(const std::string &s, char delimiter)
{
    std::vector<T> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter))
    {
        if (token.find_first_not_of(' ') != std::string::npos)
        {
            tokens.push_back(std::stof(token));
        }
    }
    return tokens;
}

std::vector<std::string> split(const std::string &s, char delimiter = ',');

std::string getISOTimeString();

std::string getMilliSecondsString();

inline double constrainAngle180(double x)
{
    x = fmod(x + 180, 360);
    if (x < 0)
        x += 360;
    return x - 180;
}

inline double constrainAngle360(double x)
{
    x = fmod(x, 360);
    if (x < 0)
        x += 360;
    return x;
}

template <typename T>
bool getNextDataEntry(std::istream &inputPoseFile, T &val)
{
    if (inputPoseFile.good())
    {
        std::string poseStr;
        std::getline(inputPoseFile, poseStr);
        if (inputPoseFile.good())
        {
            std::stringstream line(poseStr);
            line >> val;
            // return val.isGood();
            return true;
        }
    }
    return false;
}

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

template <typename T>
class SimpleHistogram{
private:
    long binCount;
    T minRange;
    T maxRange;
    T binSize;
    std::vector<T> histogram;
    size_t totalElemCount;

public:
    SimpleHistogram(int bins, T min, T max):
    binCount(bins),
    minRange(min),
    maxRange(max),
    binSize((max-min)/(binCount - 1)),
    histogram(binCount),
    totalElemCount(0){

    }

    void update(const std::vector<T> & vals){
        for(const auto & elem : vals){
            if(std::isnan(elem) || std::isinf(elem)){
                continue;
            }
            int binIdx = std::round((elem - minRange)/ binSize);
            if(binIdx < 0){
                binIdx = 0;
            }else if(binIdx >= binCount){
                binIdx = binCount - 1;
            }
            ++histogram[binIdx];
            ++totalElemCount;
        }
    }

    /**
     * @brief getPercentileBounds
     * @param percentile (approached from both sides until this is exceeded)
     * @return min-max percentile bounds
     */
    std::pair<T, T> getPercentileBounds(const T percentile) const {
              std::cout << std::setprecision(10);
        std::pair<T, T> bounds;
        T currentTotal = 0;
        bool lowerBoundFound = false;

        for(int i=0; i< binCount; ++i){
            auto bin = ((T)i * binSize + minRange);
            auto & val = histogram[i];

            currentTotal += (val/ totalElemCount);
            if(!lowerBoundFound && currentTotal > percentile){
                bounds.first = bin;
                lowerBoundFound = true;
            }else if(currentTotal > (1.0 - percentile)){
                bounds.second = bin;
                break;
            }
        }
        return bounds;
    }

    inline friend std::ostream &operator<<(std::ostream &out, const SimpleHistogram &hist){
        auto old_precision = out.precision();
        out << std::setprecision(10);
        for(int i=0; i< hist.binCount; ++i){
            out << ((T)i * hist.binSize + hist.minRange) << ", " << hist.histogram[i]/hist.totalElemCount << std::endl;
        }
        // do stuff
        out.precision(old_precision);
        return out;
    }

};

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

} // namespace utils
