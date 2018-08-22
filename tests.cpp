/**
 * This is a small set of tests to verify if things work as they should.
 * 
 */

#include <iostream>
#include <fstream>
#include <thread>
#include <algorithm>
#include <numeric>

#include <Data/CameraMatrix.hpp>
#include <Modules/NaoProvider.h>
#include <Modules/Configuration/Configuration.h>
#include <Modules/Configuration/UnixSocketConfig.hpp>
#include <Modules/Poses.h>

#include <Tools/Storage/Image.hpp>
#include <Tools/Storage/Image422.hpp>
#include <Tools/Kinematics/KinematicMatrix.h>
#include <Hardware/RobotInterface.hpp>

#include "NaoPoseInfo.hpp"
#include "NaoStability.hpp"
#include "NaoTorsoPose.hpp"
#include "ObservationSensitivityProvider.hpp"

class Configuration;
class TUHH
{

  public:
    Configuration config_;
    TUHH(std::string fileRoot) : config_(fileRoot)
    {
        NaoInfo info;
        info.bodyVersion = NaoVersion::V5;
        info.headVersion = NaoVersion::V5;
        info.bodyName = "default";
        info.headName = "default";

        config_.setLocationName("default");
        config_.setNaoHeadName(info.headName);
        config_.setNaoBodyName(info.bodyName);

        NaoProvider::init(config_, info);
        std::cout << "initialize TUHH" << std::endl;
        std::cout << Poses::init(fileRoot) << std::endl;
    }
};

int main(int argc, char **argv)
{

    std::string confRoot((argc > 1 ? argv[1] : "../../nao/home/"));
    std::string inFileName((argc > 2 ? argv[2] : "out"));

    TUHH tuhhInstance(confRoot);

    Vector2f fc, cc, fov;

    tuhhInstance.config_.mount("Projection", "Projection.json", ConfigurationType::HEAD);
    tuhhInstance.config_.get("Projection", "top_fc") >> fc;
    tuhhInstance.config_.get("Projection", "top_cc") >> cc;
    tuhhInstance.config_.get("Projection", "fov") >> fov;

    Vector2i imSize(640, 480);

    size_t maxGridPointsPerSide = 5;

    std::cout << "init" << std::endl;
    std::vector<ObservationSensitivity> sensitivitues = ObservationSensitivityProvider::getSensitivityProviders(
        1, imSize, fc, cc, fov, maxGridPointsPerSide, 0.1);

    ObservationSensitivity obs = sensitivitues[0];
    std::vector<float> jointAngles = Poses::getPose(Poses::READY);

    /**
     *  Test for sensitivity calculation
     */
    SUPPORT_FOOT sf = SUPPORT_FOOT::SF_DOUBLE;
    SENSOR_NAME sensorName = SENSOR_NAME::BOTTOM_CAMERA;
    // Camera cameraName = (sensorName == SENSOR_NAME::TOP_CAMERA) ? Camera::TOP : Camera::BOTTOM;

    jointAngles[JOINTS::JOINT::HEAD_PITCH] = 20 * TO_RAD;
    obs.updateState(jointAngles, sf, sensorName);

    // Update camera matrix.. & get baseline grid
    KinematicMatrix supFoot = NaoSensorDataProvider::getSupportFootMatrix(jointAngles, sf);

    std::vector<Vector2f> grid = obs.getGroundGrid();
    std::vector<float> baseLinePoints = obs.robotToPixelMulti(grid);

    bool observed = false;

    // test head pitch sensitivity.
    Vector3f sensitivity = obs.getSensitivityForJointForCamera(JOINTS::JOINT::HEAD_PITCH, jointAngles, supFoot, grid, baseLinePoints, sensorName, observed);
    std::cout << "Test for Head pitch\n"
              << sensitivity << "\n "
              << "Observed? " << observed << std::endl;
    observed = false;
    // // test head yaw sensitivity.
    sensitivity = obs.getSensitivityForJointForCamera(JOINTS::JOINT::HEAD_YAW, jointAngles, supFoot, grid, baseLinePoints, sensorName, observed);
    std::cout << "Test for Head yaw \n"
              << sensitivity << "\n "
              << "Observed? " << observed << std::endl;

    return 0;
}
