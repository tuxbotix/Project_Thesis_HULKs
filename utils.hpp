#pragma once

#include <vector>
#include <sstream>
#include <iostream>
#include <fstream>
#include <atomic>

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
void commitToStream(std::vector<std::vector<T>> &poseList, std::ostream &outStream)
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
}

std::vector<std::string> split(const std::string &s, char delimiter = ',')
{
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter))
    {
        tokens.push_back(token);
    }
    return tokens;
}

std::vector<float> splitToNumbers(const std::string &s, char delimiter = ',')
{
    std::vector<float> tokens;
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
