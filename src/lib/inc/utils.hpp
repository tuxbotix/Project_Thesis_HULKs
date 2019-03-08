#pragma once

#include <algorithm>
#include <atomic>
#include <cmath>
#include <fstream>
#include <future>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <vector>

#define TO_RAD_DBL static_cast<double>(M_PI / 180.0)
#define TO_RAD_FLT static_cast<float>(M_PI / 180.0)

namespace utils {
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
void commitToStream(std::vector<T> &poseList, std::ostream &outStream) {
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
  for (const auto &p : poseList) {
#if USE_STRINGSTREAM
    buffer << p << "\n";
    if (buffer.tellg() > 4E6) { // write each 4MB :P
      std::cout << "buf write";
      outStream << buffer.str();
      buffer.str(std::string());
      buffer.clear();
    }
#else
    outStream << p << "\n";
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
void commitToStreamVec(std::vector<T> &poseList, std::ostream &outStream) {
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
  for (const auto &i : poseList) {
    for (const auto &angle : i) {
#if USE_STRINGSTREAM
      buffer << angle << " ";
#else
      outStream << angle << " ";
#endif
    }
#if USE_STRINGSTREAM
    buffer << "\n";
    if (buffer.tellg() > 4E6) { // write each 4MB :P
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
std::vector<T> splitToNumbers(const std::string &s, char delimiter) {
  std::vector<T> tokens;
  std::string token;
  std::istringstream tokenStream(s);
  while (std::getline(tokenStream, token, delimiter)) {
    if (token.find_first_not_of(' ') != std::string::npos) {
      tokens.push_back(std::stof(token));
    }
  }
  return tokens;
}

std::vector<std::string> split(const std::string &s, char delimiter = ',');

std::string getISOTimeString();

std::string getMilliSecondsString();

template <typename T> inline T constrainAngle180(const T &x) {
  T v = fmod(x + 180, 360);
  if (v < 0)
    v += 360;
  return v - 180;
}

template <typename T> inline double constrainAngle360(const T &x) {
  T v = fmod(x, 360);
  if (v < 0)
    v += 360;
  return v;
}

template <typename T>
bool getNextDataEntry(std::istream &inputPoseFile, T &val) {
  if (inputPoseFile.good()) {
    std::string poseStr;
    std::getline(inputPoseFile, poseStr);
    if (inputPoseFile.good()) {
      std::stringstream line(poseStr);
      line >> val;
      // return val.isGood();
      return true;
    }
  }
  return false;
}

template <typename T> class SimpleHistogram {
private:
  long binCount;
  T minRange;
  T maxRange;
  T binSize;
  std::vector<T> histogram;
  size_t totalElemCount;

public:
  SimpleHistogram(int bins, T min, T max)
      : binCount(bins), minRange(min), maxRange(max),
        binSize((max - min) / (binCount - 1)), histogram(binCount),
        totalElemCount(0) {}

  void update(const std::vector<T> &vals) {
    for (const auto &elem : vals) {
      if (std::isnan(elem) || std::isinf(elem)) {
        continue;
      }
      int binIdx = std::round((elem - minRange) / binSize);
      if (binIdx < 0) {
        binIdx = 0;
      } else if (binIdx >= binCount) {
        binIdx = binCount - 1;
      }
      ++histogram[binIdx];
      ++totalElemCount;
    }
  }

  T getPercentAboveAbsValue(const T &val) const {
    T count = 0;
    for (int i = 0; i < binCount; ++i) {
      auto bin = std::fabs((T)i * binSize + minRange);
      if (bin > val) {
        count += histogram[i];
      }
    }
    return count * 100 / binCount;
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

    for (int i = 0; i < binCount; ++i) {
      auto bin = ((T)i * binSize + minRange);
      auto &val = histogram[i];

      currentTotal += (val / totalElemCount);
      if (!lowerBoundFound && currentTotal > percentile) {
        bounds.first = bin;
        lowerBoundFound = true;
      } else if (currentTotal > (1.0 - percentile)) {
        bounds.second = bin;
        break;
      }
    }
    return bounds;
  }

  inline friend std::ostream &operator<<(std::ostream &out,
                                         const SimpleHistogram &hist) {
    auto old_precision = out.precision();
    out << std::setprecision(10);
    for (int i = 0; i < hist.binCount; ++i) {
      out << ((T)i * hist.binSize + hist.minRange) << ", "
          << hist.histogram[i] / hist.totalElemCount << "\n";
    }
    out.flush();
    // do stuff
    out.precision(old_precision);
    return out;
  }
};

template <typename T>
/**
 * @brief sgn
 * @param val
 * @return
 */
int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

/**
 * @brief getSmallerTriangleNum Get the next (smaller) sized triangle num
 * @param sizeOfSize
 * @return
 */
inline size_t getSmallerTriangleNum(const size_t &sizeOfSize) {
  return sizeOfSize * (sizeOfSize - 1) / 2;
}

/**
 * @brief getTriangleNum
 * @param sizeOfSize
 * @return
 */
inline size_t getTriangleNum(const size_t &sizeOfSize) {
  return sizeOfSize * (sizeOfSize + 1) / 2;
}

/**
 * @brief getClosestTriangleNum
 * Source:
 * https://stackoverflow.com/questions/23319520/fastest-way-to-find-nearest-triangle-number
 * @param index
 * @return
 */
template <typename T = size_t>
inline std::pair<size_t, T> getClosestTriangleNumAndBase(const T &index) {
  // solve it for the explicit formula
  size_t m = static_cast<size_t>(
      std::floor(0.5 * (-1.0 + sqrt(1.0 + 8.0 * static_cast<double>(index)))));
  size_t tan0 = (m * m + m) / 2;         // the closest one is either this
  size_t tan1 = (m * m + 3 * m + 2) / 2; // or this

  if (index > tan1) { // tan1 is larger, so eval it first
    return {tan1, m + 1};
  } else if (index >= tan0) {
    return {tan0, m};
  } else {
    std::cerr << "Can't find nearest triangle!!! " << index << " " << tan0
              << " " << tan1 << " " << m << std::endl;
    throw("Can't find nearest triangle!!!");
  }
}

/**
 * @brief getIndexUpperTriangle Get begin index to fill in the cost arr..
 * @return
 */
template <typename T = size_t>
inline T getIndexUpperTriangle(const size_t &i, const size_t &j,
                               const size_t &sideLen) {
  return static_cast<T>(((i * sideLen + j) / 2) + 1);
}

/**
 * @brief TRIANGLE_NUMS_30
 * An array of triangular numbers.
 */
static const std::array<size_t, 30> TRIANGLE_NUMS_30 = []() {
  std::array<size_t, 30> vals = {};
  for (size_t i = 0; i < vals.size(); i++) {
    vals[i] = getTriangleNum(i);
  }
  return vals;
}();

template<class T, typename F>
/**
 * @brief parallel_sort - taken from https://codereview.stackexchange.com/questions/22744/multi-threaded-sort
 * @param data
 * @param len
 * @param grainsize
 */
void parallel_sort(T begin, size_t len, size_t grainsize, F compareFunc)
{
  //  grainsize = std::min(grainsize, len / 8); // hard limit for thread count
  // Use grainsize instead of thread count so that we don't e.g.
  // spawn 4 threads just to sort 8 elements.
  if (len <= grainsize) {
#if DEBUG_IN_THREAD
    {
      std::lock_guard<std::mutex> lg(mtx_cout_);
      std::cout << "Starting thread" << std::endl;
    }
#endif
    std::sort(begin, begin + len, compareFunc);
  } else {

    ptrdiff_t lenHalf = len / 2;
    auto future =
        std::async(parallel_sort<T, F>, begin, lenHalf, grainsize, compareFunc);
    // No need to spawn another thread just to block the calling
    // thread which would do nothing.
    //#if DEBUG_IN_THREAD
    //    {
    //      std::lock_guard<std::mutex> lg(mtx_cout_);
    //      std::cout << "Starting thread" << std::endl;
    //    }
    //#endif
    auto compareFuncCopy = compareFunc;
    parallel_sort(begin + lenHalf, (len - lenHalf), grainsize, compareFuncCopy);

    future.wait();

    std::inplace_merge(begin, begin + lenHalf, begin + len, compareFunc);
  }
}

// template<class T, typename F>
///**
// * @brief parallel_sort - taken from
// https://codereview.stackexchange.com/questions/22744/multi-threaded-sort
// * @param data
// * @param len
// * @param grainsize
// */
// void parallelOp(T* data, int len, int grainsize, F&& compareFunc)
//{
//  grainsize = std::min(grainsize, len / 8); // hard limit for thread count
//  // Use grainsize instead of thread count so that we don't e.g.
//  // spawn 4 threads just to sort 8 elements.
//  if (len < grainsize) {
//    std::sort(data, data + len, compareFunc);
//  } else {
//#if DEBUG_IN_THREAD
//    {
//      std::lock_guard<std::mutex> lg(mtx_cout_);
//      std::cout << "Starting thread" << std::endl;
//    }
//#endif
//    auto future = std::async(parallel_sort<T>, data, len / 2, grainsize);
//// No need to spawn another thread just to block the calling
//// thread which would do nothing.
//#if DEBUG_IN_THREAD
//    {
//      std::lock_guard<std::mutex> lg(mtx_cout_);
//      std::cout << "Starting thread" << std::endl;
//    }
//#endif
//    parallel_sort(data + len / 2, len / 2, grainsize);

//    future.wait();

//    std::inplace_merge(data, data + len / 2, data + len, compareFunc);
//  }
//}

} // namespace utils
