#pragma once

#include <memory>

#include <Tools/Math/Eigen.hpp>

#include "utils.hpp"

namespace CalibrationFeatures {

template <typename T = float> class CalibrationFeature {
public:
  CalibrationFeature() {}

  virtual void updatePointPose(const Vector2<T> &position,
                               const T &orientation) = 0;
  virtual void updatePointPose(const Vector3<T> &position,
                               const Vector3<T> &orientation) = 0;
  virtual VecVector2<T> getGroundPoints() const = 0;
  virtual VecVector3<T> get3DPoints() const = 0;
};

// template <class T> using Residual = std::vector<T>;

template <class T>
using CalibrationFeaturePtr =
    std::shared_ptr<CalibrationFeatures::CalibrationFeature<T>>;

template <typename T = float> class GroundGrid : public CalibrationFeature<T> {
private:
  VecVector2<T> baseGrid2D;   // relative to self-origin
  VecVector2<T> gridPoints2D; // relative to self-origin
public:
  GroundGrid(const size_t maxGridPointsPerSide, const float gridSpacing,
             const Vector2<T> &position = {0, 0}, const T &orientation = 0)
      : CalibrationFeature<T>() {
    T x, y;
    const int gridSizeHalf = maxGridPointsPerSide / 2;
    for (size_t i = 0; i < maxGridPointsPerSide; i++) {
      x = (static_cast<int>(i) - gridSizeHalf) * gridSpacing;
      for (size_t j = 0; j < maxGridPointsPerSide; j++) {
        y = (static_cast<int>(j) - gridSizeHalf) * gridSpacing;
        baseGrid2D.emplace_back(x, y);
      }
    }
    gridPoints2D.resize(baseGrid2D.size());
    updatePointPose(position, orientation);
  }

  void updatePointPose(const Vector2<T> &position, const T &orientation) {
    Eigen::Rotation2D<T> rot2(orientation);
    for (size_t i = 0; i < baseGrid2D.size(); ++i) {
      gridPoints2D[i] = position + (rot2 * baseGrid2D[i]);
    }
  }

  void updatePointPose(const Vector3<T> &position,
                       const Vector3<T> &orientation) {
    updatePointPose(Vector2<T>(position.x(), position.y()), orientation.y());
  }

  VecVector3<T> get3DPoints() const {
    VecVector3<T> output;
    for (const auto &val : gridPoints2D) {
      output.emplace_back(val.x(), val.y(), T(0));
    }
    return output;
  }

  VecVector2<T> getGroundPoints() const { return gridPoints2D; }
};

template <typename T = float>
class ChessBoardPattern : public CalibrationFeature<T> {
private:
protected:
  int width;  // width of grid
  int height; // height of grid
  int squareSize;

  VecVector3<T> baseGrid;   // relative to self-origin
  VecVector3<T> gridPoints; // relative to self-origin

public:
  ChessBoardPattern(const int &width, const int &height, const T &squareSize,
                    const Vector3<T> &position, const Vector3<T> &orientation)
      : CalibrationFeature<T>(), width(width), height(height),
        squareSize(squareSize), gridPoints() {
    // defaults to points in x-y plane
    T wHalfInUnits = (T(width) * squareSize) / T(2);
    T hHalfInUnits = (T(height) * squareSize) / T(2);
    for (size_t w = 0; w < width; w++) {
      for (size_t h = 0; h < height; h++) {
        baseGrid.push_back(Vector3<T>((T(w) * squareSize) - wHalfInUnits,
                                      (T(h) * squareSize) - hHalfInUnits,
                                      T(0.0)));
      }
    }

    gridPoints.resize(baseGrid.size());
    updatePointPose(position, orientation);
  }

  void updatePointPose(const Vector3<T> &position,
                       const Vector3<T> &orientation) {
    std::cerr << " ChessBoardPattern 3D not implemented" << std::endl;
    //    auto &_gridPts = baseGrid;

    //    Eigen::Translation<T, 2> trans(position);
    //    Eigen::Rotation2D<T> rot2(orientation * TO_RAD_DBL);

    //    Vector2<T> tempPt;
    //    for (size_t i = 0; i < _gridPts.size(); ++i) {
    //      tempPt = trans * rot2 * Vector2<T>(_gridPts[i].x(),
    //      _gridPts[i].y()); gridPoints[i] = {tempPt.x(), tempPt.y(), T(0)};
    //    }
  }

  void updatePointPose(const Vector2<T> &position, const T &orientation) {
    std::cerr << " ChessBoardPattern 2D not implemented" << std::endl;
  }

  VecVector3<T> get3DPoints() const { return ChessBoardPattern<T>::gridPoints; }

  VecVector2<T> getGroundPoints() const {
    VecVector2<T> output(gridPoints.size());
    for (const auto &pt : gridPoints) {
      output.emplace_back(pt.x(), pt.y());
    }
    return output;
  }
};

template <typename T = float>
class ChessBoardPatternOnFloor : public ChessBoardPattern<T> {
private:
  VecVector2<T> gridPoints; // relative to self-origin
public:
  ChessBoardPatternOnFloor(const int &width, const int &height,
                           const T &squareSize, const Vector2<T> &position,
                           const T &orientation)
      : ChessBoardPattern<T>(width, height, squareSize,
                             {position.x(), position.y(), 0},
                             {0, orientation, 0}) {
    gridPoints.resize(ChessBoardPattern<T>::baseGrid.size());
    updatePointPose(position, orientation);
//    std::cout << "Construction done" << std::endl;
  }

  void updatePointPose(const Vector2<T> &position, const T &orientation) {
    auto &_gridPts = ChessBoardPattern<T>::baseGrid;

    Eigen::Translation<T, 2> trans(position);
    Eigen::Rotation2D<T> rot2(orientation * TO_RAD_DBL);

    Vector2<T> tempPt;
    for (size_t i = 0; i < _gridPts.size(); ++i) {
      tempPt = trans * rot2 * Vector2<T>(_gridPts[i].x(), _gridPts[i].y());
      gridPoints[i] = {tempPt.x(), tempPt.y()};
//      std::cout << _gridPts[i].x() << " " << _gridPts[i].y() << std::endl;
    }
  }

  void updatePointPose(const Vector3<T> &position,
                       const Vector3<T> &orientation) {
    std::cerr << " ChessBoardPatternOnFloor 3D not implemented" << std::endl;
    updatePointPose(Vector2<T>(position.x(), position.y()), orientation.y());
  }

  VecVector3<T> get3DPoints() const {
//    std::cout << "get points 3d chess" << std::endl;
    return ChessBoardPattern<T>::get3DPoints();
  }

  VecVector2<T> getGroundPoints() const {
//    std::cout << "get points floor chess " << gridPoints.size() << std::endl;
    return gridPoints;
  }
};

} // namespace CalibrationFeatures
