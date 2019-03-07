#pragma once

#include <Tools/Math/Eigen.hpp>

#include "utils.hpp"

template <typename T = float> class CalibrationFeature {
public:
  CalibrationFeature() {}
};

template <typename T = float> class GroundGrid : public CalibrationFeature<T> {
public:
  GroundGrid() {}
};

template <typename T = float>
class ChessBoardPattern : public CalibrationFeature<T> {
private:
protected:
  int width;  // width of grid
  int height; // height of grid
  int squareSize;

  VecVector3<T> gridPoints; // relative to self-origin

public:
  ChessBoardPattern(const int &width, const int &height, const T &squareSize)
      : width(width), height(height), squareSize(squareSize), gridPoints() {
    // defaults to points in x-y plane
    auto wHalfInUnits = width * squareSize / T(2);
    auto hHalfInUnits = height * squareSize / T(2);
    for (size_t w = 0; w < width; w++) {
      for (size_t h = 0; h < height; h++) {
        gridPoints.push_back(Vector3<T>(w * squareSize - wHalfInUnits,
                                        h * squareSize - hHalfInUnits, T(0.0)));
      }
    }
  }

  VecVector3<T> getTransformedPoints(const Vector3<T> &position,
                                     const Vector3<T> &orientation) {}
};

template <typename T = float>
class ChessBoardPatternOnFloor : public ChessBoardPattern<T> {
public:
  ChessBoardPatternOnFloor(const int &width, const int &height,
                           const T &squareSize)
      : ChessBoardPattern<T>(width, height, squareSize) {}

  VecVector2<T> getTransformedPoints(const Vector2<T> &position,
                                     const T &orientation) {
    auto &_gridPts = ChessBoardPattern<T>::gridPoints;

    Eigen::Translation<T, 2> trans(position);
    Eigen::Rotation2D<T> rot2(orientation * TO_RAD_DBL);
    VecVector2<T> output(_gridPts.size());
    for (size_t i = 0; i < output.size(); ++i) {
      output[i] = trans * rot2 * Vector2<T>(_gridPts[i].x(), _gridPts[i].y());
    }
    return output;
  }
};
