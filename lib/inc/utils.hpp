#pragma once

#include <vector>
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
        buffer << p;
#else
        outStream << p;
#endif
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

} // namespace utils
