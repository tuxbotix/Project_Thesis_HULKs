#include "CameraObservationModel.hpp"
#include "Solvers.hpp"
#include "utils.hpp"

/**
 * To be precise, this give sensitivity of camera observation :P
 */
const std::string CameraObservationModel::name = "CameraObservationModel";
CameraObservationModel::CameraObservationModel(const Vector2i &imSize, const Vector2f &fc, const Vector2f &cc, const Vector2f &fov,
                                               uint64_t dimensionExtremum, const size_t &maxGridPointsPerSide, const float &gridSpacing)
    : // maxGridPointsPerSide(maxGridPointsPerSide),
      maxValPerDim(dimensionExtremum),
      imSize(imSize),
      // gridSpacing(gridSpacing),
      deltaThetaCorse(5.0f * TO_RAD), deltaThetaFine(1.0f * TO_RAD), horizon(0)
{
  camMat.fc = fc;
  camMat.fc.x() *= imSize.x();
  camMat.fc.y() *= imSize.y();
  camMat.cc = cc;
  camMat.cc.x() *= imSize.x();
  camMat.cc.y() *= imSize.y();
  camMat.fov = fov;
  // Default state, ready pose and top camera with double foot.
  // updateState(Poses::getPose(Poses::READY), SUPPORT_FOOT::SF_LEFT, SENSOR_NAME::TOP_CAMERA);

  float x, y;
  const int gridSizeHalf = maxGridPointsPerSide / 2;
  for (size_t i = 0; i < maxGridPointsPerSide; i++)
  {
    x = ((int)i - gridSizeHalf) * gridSpacing;
    for (size_t j = 0; j < maxGridPointsPerSide; j++)
    {
      y = ((int)j - gridSizeHalf) * gridSpacing;
      basicGrid.emplace_back(x, y);
    }
  }
}

/**
   * Update the state of the robot.
   */
void CameraObservationModel::updateState(const rawPoseT &jointAngles, const SUPPORT_FOOT &sf, const SENSOR_NAME &sensorNames)
{
  Camera camName = sensorNames == SENSOR_NAME::TOP_CAMERA ? Camera::TOP : Camera::BOTTOM;
  NaoSensorDataProvider::updatedCameraMatrix(jointAngles, sf, camMat, camName);
  // update default horizon
  horizon = std::min(std::min(camMat.getHorizonHeight(0), camMat.getHorizonHeight(imSize.x() - 1)), imSize.y() - 1);
}

/**
 * Get 3D grid with given grid spacing relative to robot's ground coord.
 * Obviously, camera matrix must be updated before calling this.
 * 1. This has a hard range limit of 2m
 */
// TODO  Make this private.
std::vector<Vector2f> CameraObservationModel::getGroundGrid(bool &success)
{
  std::vector<Vector2f> output;
  Vector2f robotCoords;

  // Check if horizon line is above bottom of camera view.
  bool proceed = horizon < (imSize.y() - 1);

  if (proceed)
  {
    proceed = (camMat.pixelToRobot(camMat.cc.cast<int>(), robotCoords) && robotCoords.norm() <= maxViewDist);
    // std::cout << "CamCenterRayToGround: " << robotCoords.x() << ", " << robotCoords.y() << std::endl;
  }
#if DEBUG_CAM_OBS
  else
  {
    std::cout << "HorizA" << camMat.horizonA << " horizB " << camMat.horizonB << std::endl;
    std::cout << " Out of horizon!" << std::endl;
    std::cout << camMat.pixelToRobot(Vector2i(0, 0), robotCoords) << "(" << robotCoords.x() << ", " << robotCoords.y() << ")\t";
    std::cout << camMat.pixelToRobot(Vector2i(imSize.x(), 0), robotCoords) << "(" << robotCoords.x() << ", " << robotCoords.y() << ")" << std::endl;
    std::cout << camMat.pixelToRobot(Vector2i(0, imSize.y()), robotCoords) << "(" << robotCoords.x() << ", " << robotCoords.y() << ")\t";
    std::cout << camMat.pixelToRobot(imSize, robotCoords) << "(" << robotCoords.x() << ", " << robotCoords.y() << ")" << std::endl;
  }
#endif
  if (proceed)
  {
    Vector2f tempCoord;
    // Eigen::Translation<float, 2> trans(tempCoord.x(), tempCoord.y());
    float theta = std::atan2(robotCoords.y(), robotCoords.x());
    Eigen::Rotation2D<float> rot2(theta);
    for (const auto &i : basicGrid)
    {
      tempCoord = robotCoords + (rot2 * i);
      output.push_back(tempCoord);
    }
  }
#if DEBUG_CAM_OBS
  else
  {
    std::cout << "CamCenterRayToGround: " << robotCoords.norm() << " " << robotCoords.x() << ", " << robotCoords.y() << std::endl;
    std::cout << "Too far view dist!" << std::endl;
  }
#endif
  success = proceed;
  return output;
}

/**
 * Robot to pixel, multiple point support
 */
std::vector<float> CameraObservationModel::robotToPixelMulti(const std::vector<Vector2f> &groundPoints)
{
  std::vector<float> output;
  Vector2i point(0.0, 0.0);
  for (const auto &i : groundPoints)
  {
    camMat.robotToPixel(i, point);
    output.push_back(point.x());
    output.push_back(point.y());
  }
  return output;
}

/**
 * Get observability (sensitivity?) of a given joint for a given camera at a given pose
 */
Vector3f CameraObservationModel::getSensitivityForJointForCamera(const JOINTS::JOINT &joint, const rawPoseT &jointAngles, const SUPPORT_FOOT &sf,
                                                                 const std::vector<Vector2f> &grid, const std::vector<float> &baselinePoints,
                                                                 const SENSOR_NAME &sensorName, bool &observed)
{
  Vector3f output(0, 0, 0);

  observed = false;
  rawPoseT tempJoints(jointAngles);

  tempJoints[joint] += deltaThetaFine;
  updateState(tempJoints, sf, sensorName);

  std::vector<float> observedPoints = robotToPixelMulti(grid);

  // int status =
  ObservationSolvers::get2dPose(baselinePoints, observedPoints, output);
  // std::cout << status << std::endl;
  // simple check.
  // observed = (Vector2f(output.x(), output.y()).norm() > 2 || abs(output.z()) > 3);

  /* constrain output +-maxValPerDim and norm(output) <= abs(maxValPerDim)
   * 1st scale each dimension to +- maxValPerDim
   * 2nd Normalize and multiply by 1000 :P
   * This is written in a strange way, to reduce loosing too much precision.
   */
  output.x() = std::floor(output.x() * maxValPerDim / imSize.x());
  output.y() = std::floor(output.y() * maxValPerDim / imSize.y());
  output.z() = std::floor(maxValPerDim * utils::constrainAngle180(output.z() / 180));

  observed = true;
  return output;
}

/**
 * Get observability (sensitivity?) of each joint for a given camera at a given pose
 */
PoseSensitivity<Vector3f> CameraObservationModel::getSensitivitiesForCamera(const rawPoseT &jointAngles, const SUPPORT_FOOT &sf, const SENSOR_NAME &sensorName)
{
  if (sensorName != SENSOR_NAME::TOP_CAMERA && sensorName != SENSOR_NAME::BOTTOM_CAMERA)
  {
    throw "Invalid sensor, not camera!";
  }

  PoseSensitivity<Vector3f> output(sensorName);
  // KinematicMatrix supportFoot2Torso = NaoSensorDataProvider::getSupportFootMatrix(jointAngles, sf);

  /// Make the basline set of points.
  updateState(jointAngles, sf, sensorName);
  bool success;
  const std::vector<Vector2f> grid = getGroundGrid(success);
  // std::cout << " success? " << success << std::endl;
  if (success)
  {
    const std::vector<float> baseLinePoints = robotToPixelMulti(grid);

    /**
     * TODO There is one major issue; The sensitivity is *ignored* based on which support foot!!!
     * ie: if double or left sup, and right leg joints are tweaked, that's completely ignored..
     * This only matters for double foot..
     */
    bool observed = false;
    if (sf == SUPPORT_FOOT::SF_LEFT || sf == SUPPORT_FOOT::SF_DOUBLE)
    {
      for (const auto &i : Sensor::CAM_OBS_L_SUP_FOOT)
      {
        // Same as..
        // vec3f p = getSensitivityForJointForCamera(i, jointAngles, supFoot, grid, baseLinePoints, cameraName, observed);
        // output.setSensitivity(i, p, observed);
        Vector3f sensitivity = getSensitivityForJointForCamera(i, jointAngles, SUPPORT_FOOT::SF_LEFT, grid, baseLinePoints, sensorName, observed);
        output.setSensitivity(i, sensitivity, observed);
      }
    }
    if (sf == SUPPORT_FOOT::SF_RIGHT || sf == SUPPORT_FOOT::SF_DOUBLE)
    {
      for (const auto &i : Sensor::CAM_OBS_R_SUP_FOOT)
      {
        // temp fix in injecting support foot as right always.
        Vector3f sensitivity = getSensitivityForJointForCamera(i, jointAngles, SUPPORT_FOOT::SF_RIGHT, grid, baseLinePoints, sensorName, observed);
        output.setSensitivity(i, sensitivity, observed);
      }
    }
  }
  return output;
}

/**
   * Get sensitivities for each joint and each sensor
   */
std::vector<PoseSensitivity<Vector3f>> CameraObservationModel::getSensitivities(const rawPoseT &jointAngles, const SUPPORT_FOOT &sf, const std::vector<SENSOR_NAME> &sensorNames)
{
  std::vector<PoseSensitivity<Vector3f>> output;
  for (const auto &i : sensorNames)
  {
    if (i == SENSOR_NAME::TOP_CAMERA || i == SENSOR_NAME::BOTTOM_CAMERA)
    {
      output.push_back(getSensitivitiesForCamera(jointAngles, sf, i));
    }
    else
    {
      std::cerr << "???" << std::endl;
      throw "Sensor Sensitivity Function not implemented.";
    }
  }
  return output;
}
