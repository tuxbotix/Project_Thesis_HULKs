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

#include "TUHHMin.hpp"
#include "NaoPoseInfo.hpp"
#include "NaoStability.hpp"
#include "NaoTorsoPose.hpp"
#define DEBUG_CAM_OBS 1
#include "ObservationSensitivityProvider.hpp"

bool poseSensitivityStreamingTest()
{
    std::vector<std::string> pOldLst = {"SENS 1535657042008010048220 1 3 8 3833 -7828 -24 9 -846 1313 -143 11 1346 -35 300 12 1341 -55 302 13 1347 -69 302 ",
                                        "SENS 1535657042008010048220 0 3 "};

    // No trailing space at end of entry.
    std::vector<std::string> pNewLst = {"SENS 1535657042008010048220 1 3 8 3833 -7828 -24 9 -846 1313 -143 11 1346 -35 300 12 1341 -55 302 13 1347 -69 302",
                                        "SENS 1535657042008010048220 0 3"};
    bool finalSuccess = true;
    /// Test if old space-trailing format can be read
    for (size_t i = 0; i < pOldLst.size(); i++)
    {
        PoseSensitivity<Vector3f> sens;
        std::stringstream sso(pOldLst[i]);
        std::stringstream ssi;

        sso >> sens;
        ssi << sens;
        // Old string must convert to new strings.
        bool success1 = pNewLst[i].compare(ssi.str()) == 0;
        if (!success1)
        {
            std::cout << ssi.str() << std::endl;
        }
        std::cout << "PoseSensitivity Istreamed vs Ostreamed compare [old -> new] " << std::to_string(i) << " : " << (success1 ? "success" : "fail") << std::endl;
        finalSuccess &= success1;
    }
    /// Test if new format can be read
    for (size_t i = 0; i < pNewLst.size(); i++)
    {
        PoseSensitivity<Vector3f> sens;
        std::stringstream sso(pNewLst[i]);
        std::stringstream ssi;

        sso >> sens;
        ssi << sens;
        // Old string must convert to new strings.
        bool success1 = pNewLst[i].compare(ssi.str()) == 0;
        if (!success1)
        {
            std::cout << ssi.str() << std::endl;
        }
        std::cout << "PoseSensitivity Istreamed vs Ostreamed compare [new -> new] " << std::to_string(i) << " : " << (success1 ? "success" : "fail") << std::endl;
        finalSuccess &= success1;
    }
    return finalSuccess;
}
bool poseStreamingTest()
{
    NaoPoseAndRawAngles<float> poseAndAngles1;
    std::string p = "NaoPoseAndRawAngles NaoPose someIdlalala 0 -72 0 -0.1 -0.1 0.225 6 -12 0 -0.04 -0.1 0 0 0 0 -1.25664 0 "
                    "1.5708 0.2 1.5708 -0.00872665 0 0 0.0895838 0.274045 -1.33344 1.64128 -0.157236 -0.384673 0.0895838 0.277598 "
                    "-1.30291 2.00971 -0.543341 -0.365134 1.5708 -0.2 -1.5708 0.00872665 0 0";
    std::stringstream ss(p);
    ss >> poseAndAngles1;
    // std::cout << "PoseandAngles Istream" << std::endl;
    // std::cout << poseAndAngles1 << std::endl;
    // std::cout << std::endl;
    // std::cout << "PoseandAngles Ostream" << std::endl;
    // NaoPoseAndRawAngles<float> poseAndAngles(pose, joints);
    // std::cout << poseAndAngles1 << std::endl;
    // std::cout << std::endl;
    std::stringstream ss1;
    ss1 << poseAndAngles1;
    bool success = p.compare(ss1.str()) == 0;
    std::cout << "NaoPoseAndRaw Istreamed vs Ostreamed compare: " << (success ? "success" : "fail") << std::endl;
    return success;
}

int main(int argc, char **argv)
{

    std::string confRoot((argc > 1 ? argv[1] : "../../nao/home/"));
    std::string inFileName((argc > 2 ? argv[2] : "out"));
    std::string supportFootName((argc > 3 ? argv[3] : "d"));

    SUPPORT_FOOT supportFoot = SUPPORT_FOOT::SF_DOUBLE;
    if (supportFootName.compare("l") == 0)
    {
        supportFoot = SUPPORT_FOOT::SF_LEFT;
        std::cout << "Left Support foot." << std::endl;
    }
    else if (supportFootName.compare("r") == 0)
    {
        supportFoot = SUPPORT_FOOT::SF_RIGHT;
        std::cout << "Right Support foot." << std::endl;
    }
    else
    {
        std::cout << "Going for double foot." << std::endl;
    }

    TUHH tuhhInstance(confRoot);

    Vector2f fc, cc, fov;

    tuhhInstance.config_.mount("Projection", "Projection.json", ConfigurationType::HEAD);
    tuhhInstance.config_.get("Projection", "top_fc") >> fc;
    tuhhInstance.config_.get("Projection", "top_cc") >> cc;
    tuhhInstance.config_.get("Projection", "fov") >> fov;

    Vector2i imSize(640, 480);

    /**
     * Pose types streaming test
     */
    poseStreamingTest();

    /**
     * Pose sensitivity streaming test
     */
    poseSensitivityStreamingTest();

    /**
     * Other tests
     */
    size_t maxGridPointsPerSide = 15;

    std::cout << "init" << std::endl;
    std::vector<ObservationSensitivity> sensitivitues = ObservationSensitivityProvider::getSensitivityProviders(
        1, imSize, fc, cc, fov, 1000, maxGridPointsPerSide, 0.05);

    ObservationSensitivity obs = sensitivitues[0];
    std::vector<float> jointAngles(JOINTS::JOINT::JOINTS_MAX, 0.0);

    std::cout << NaoProvider::minRange(JOINTS::JOINT::L_KNEE_PITCH) << std::endl;
    /**
     *  Test for sensitivity calculation
     */

    SENSOR_NAME sensorName = SENSOR_NAME::BOTTOM_CAMERA;
    // Camera cameraName = (sensorName == SENSOR_NAME::TOP_CAMERA) ? Camera::TOP : Camera::BOTTOM;

    jointAngles[JOINTS::JOINT::HEAD_PITCH] = 20 * TO_RAD;
    const std::vector<float> newJoints(jointAngles);

    obs.camObsModelPtr->updateState(newJoints, supportFoot, sensorName);

    // // Update camera matrix.. & get baseline grid
    // // KinematicMatrix supFoot = NaoSensorDataProvider::getSupportFootMatrix(newJoints, sf);
    bool t;
    const std::vector<Vector2f> grid = obs.camObsModelPtr->getGroundGrid(t);
    if (!t)
    {
        std::cout << "failed to get ground grid for this sensor" << std::endl;
    }
    const std::vector<float> baseLinePoints = obs.camObsModelPtr->robotToPixelMulti(grid);

    // obs.updateState(newJoints, sf, sensorName);
    bool observed = false;

    // test head pitch sensitivity.
    Vector3f sensitivity = obs.camObsModelPtr->getSensitivityForJointForCamera(JOINTS::JOINT::L_ANKLE_ROLL, newJoints, supportFoot, grid, baseLinePoints, sensorName, observed);
    std::cout << "Test for left ankle roll pitch\n"
              << sensitivity << "\n "
              << "Observed? " << observed << std::endl;
    observed = false;
    // obs.updateState(newJoints, sf, sensorName);
    // test head yaw sensitivity.
    sensitivity = obs.camObsModelPtr->getSensitivityForJointForCamera(JOINTS::JOINT::R_ANKLE_ROLL, newJoints, supportFoot, grid, baseLinePoints, sensorName, observed);
    std::cout << "Test for right ankle roll pitch\n"
              << sensitivity << "\n "
              << "Observed? " << observed << std::endl;

    // for (auto i : grid)
    // {
    //     // std::cout << i<<std::endl;
    //     std::cout << "(" << i.x() << ", " << i.y() << ")" << std::endl;
    // }
    std::vector<PoseSensitivity<Vector3f>> sensitivityOutput =
        obs.camObsModelPtr->getSensitivities(newJoints, supportFoot, {SENSOR_NAME::BOTTOM_CAMERA});

    for (auto &i : sensitivityOutput)
    {
        Vector3f val;
        bool obs;
        for (int j = 0; j < JOINTS::JOINT::JOINTS_MAX; j++)
        {
            i.getSensitivity(static_cast<JOINTS::JOINT>(j), val, obs);
            // std::cout << "SENS-> " << j << " (" << val.x() << ", " << val.y() << ", " << val.z() << ") o:" << obs << std::endl;
            if (obs)
            {
                std::cout << "SENS-> " << j << " " << Vector2f(val.x(), val.y()).norm() << " o:" << obs << std::endl;
            }
        }
    }

    return 0;
}
