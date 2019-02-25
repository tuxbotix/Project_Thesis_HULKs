#pragma once

#include <Tools/Math/Eigen.hpp>

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
  int width;  // width of grid
  int height; // height of grid
  int squareSize;
  VecVector3<T> gridPoints; // relative to self-origin

public:
  ChessBoardPattern(const int &width, const int &height, const T &squareSize) {}

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
                                     const T &orientation) {}
};
