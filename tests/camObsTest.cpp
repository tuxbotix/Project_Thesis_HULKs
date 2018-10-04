#include <iostream>
// #include <iomanip>
#include <fstream>
#include <sstream>
#include <cmath>
#include <chrono>

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
#include "CameraObservationModel.hpp"
#include "ObservationSensitivityProvider.hpp"

typedef std::vector<std::pair<int, std::vector<float>>> JointAndPoints;

void weirdPointTest(const ObservationModelConfig cfg)
{
    // joint 12, sensor 1
    JOINTS::JOINT joint = static_cast<JOINTS::JOINT>(12);
    SENSOR_NAME sensorName = static_cast<SENSOR_NAME>(1);

    std::vector<float> jointAngles = {3.141590e-01, 1.396260e-01, 1.570800e+00, 2.000000e-01, 1.570800e+00, -8.726650e-03, 0.000000e+00,
                                      0.000000e+00, 4.345030e-02, 3.923680e-01, -1.020720e+00, 1.700120e+00, -5.993040e-01, -3.939770e-01,
                                      4.345030e-02, 1.251520e-01, -1.250350e+00, 9.796810e-01, 3.451760e-01, -1.220670e-01, 1.570800e+00,
                                      -2.000000e-01, -1.570800e+00, 8.726650e-03, 0.000000e+00, 0.000000e+00}; // stat 1;

    CameraObservationModel cObs(cfg.imSize, cfg.fc, cfg.cc, cfg.fov,
                                cfg.dimensionExtremum, cfg.maxGridPointsPerSide, cfg.gridSpacing);
    SUPPORT_FOOT sf = SUPPORT_FOOT::SF_LEFT;

    cObs.updateState(jointAngles, sf, sensorName);
    bool success;
    std::vector<Vector2f> gridSet = cObs.getGroundGrid(success);
    // std::cout << " success? " << success << std::endl;
    if (success)
    {
        std::vector<std::pair<bool, Vector2f>> baseLinePointSet = cObs.robotToPixelMulti(gridSet);
        std::vector<Vector2f> grid;
        // Use only gridPoints that made into camera plane.
        for (size_t i = 0; i < gridSet.size(); i++)
        {
            if (baseLinePointSet[i].first)
            {
                grid.push_back(gridSet[i]);
            }
        }
        bool observed;
        Vector3f sensitivity = cObs.getSensitivityForJointForCamera(joint, jointAngles, sf, grid, std::vector<Vector2f>(0), sensorName, observed);
        std::cout << sensitivity << std::endl;
    }
}

void weirdPointTest2(const ObservationModelConfig cfg)
{
    /*
    dimension Limit violation!!! j->11 0 0.628319 0.244346 1.5708 0.2 1.5708 -0.00872665 0 0 0.0197297 0.530006 -0.18356 0.940766 -0.713433 -0.373344 0.0197297 0.231953 -0.15904 0.857232 -0.661697 -0.0743088 1.5708 -0.2 -1.5708 0.00872665 0 0 stat 1

941.217
113.977
95.9851
*/
    // joint 12, sensor 1
    JOINTS::JOINT joint = static_cast<JOINTS::JOINT>(9);
    SENSOR_NAME sensorName = static_cast<SENSOR_NAME>(1);

    std::vector<float> jointAngles = {6.283190e-01, 3.839720e-01, 1.570800e+00, 2.000000e-01, 1.570800e+00, -8.726650e-03, 0.000000e+00, 0.000000e+00, -8.133260e-02, 2.935800e-01, -1.429230e+00, 1.255460e+00, 1.528070e-02, -3.516450e-01, -8.133260e-02, -5.249380e-04, -1.471790e+00, 9.030850e-01, 4.139150e-01, -4.027070e-02, 1.570800e+00, -2.000000e-01, -1.570800e+00, 8.726650e-03, 0.000000e+00, 0.000000e+00};
    if (jointAngles.size() != static_cast<size_t>(JOINTS::JOINT::JOINTS_MAX))
    {
        std::cout << "joint angle count is wrong" << std::endl;
        return;
    }

    CameraObservationModel cObs(cfg.imSize, cfg.fc, cfg.cc, cfg.fov,
                                cfg.dimensionExtremum, cfg.maxGridPointsPerSide, cfg.gridSpacing);
    SUPPORT_FOOT sf = SUPPORT_FOOT::SF_LEFT;

    std::vector<PoseSensitivity<Vector3f>> senses = cObs.getSensitivities(jointAngles, sf, {sensorName});

    for (auto &i : senses)
    {
        Vector3f val;
        bool obs;
        for (int j = 0; j < JOINTS::JOINT::JOINTS_MAX; j++)
        {
            i.getSensitivity(static_cast<JOINTS::JOINT>(j), val, obs);
            // std::cout << "SENS-> " << j << " (" << val.x() << ", " << val.y() << ", " << val.z() << ") o:" << obs << std::endl;
            if (obs)
            {
                std::cout << "SENS-> " << j << " " << val.x() << ", " << val.y() << ", " << val.z() << " o:" << obs << std::endl;
            }
        }
    }
}

void weirdPointTest2ObsProvider(const ObservationModelConfig cfg)
{
    auto obs = ObservationSensitivityProvider::getSensitivityProvider(cfg);
    JOINTS::JOINT joint = static_cast<JOINTS::JOINT>(9);

    std::vector<float> jointAngles = {6.283190e-01, 3.839720e-01, 1.570800e+00, 2.000000e-01, 1.570800e+00, -8.726650e-03, 0.000000e+00, 0.000000e+00, -8.133260e-02, 2.935800e-01, -1.429230e+00, 1.255460e+00, 1.528070e-02, -3.516450e-01, -8.133260e-02, -5.249380e-04, -1.471790e+00, 9.030850e-01, 4.139150e-01, -4.027070e-02, 1.570800e+00, -2.000000e-01, -1.570800e+00, 8.726650e-03, 0.000000e+00, 0.000000e+00};
    if (jointAngles.size() != static_cast<size_t>(JOINTS::JOINT::JOINTS_MAX))
    {
        std::cout << "joint angle count is wrong" << std::endl;
        return;
    }

    SUPPORT_FOOT sf = SUPPORT_FOOT::SF_LEFT;

    std::vector<PoseSensitivity<Vector3f>> senses = obs.getSensitivities(jointAngles, sf);

    for (auto &i : senses)
    {
        Vector3f val;
        bool obs;
        for (int j = 0; j < JOINTS::JOINT::JOINTS_MAX; j++)
        {
            i.getSensitivity(static_cast<JOINTS::JOINT>(j), val, obs);
            // std::cout << "SENS-> " << j << " (" << val.x() << ", " << val.y() << ", " << val.z() << ") o:" << obs << std::endl;
            if (obs)
            {
                std::cout << "SENS-> " << j << " " << val.x() << ", " << val.y() << ", " << val.z() << " o:" << obs << std::endl;
            }
        }
    }
}

int main(int argc, char *argv[])
{
    // std::ifstream inFile;
    // if (argc > 1)
    // {
    //     inFile.open(argv[1]);
    // }
    // if (!inFile)
    // {
    //     std::cout << "Unable to read file." << std::endl;
    //     return -1;
    // }

    std::string supportFootName((argc > 2 ? argv[2] : "d"));
    std::string confRoot((argc > 3 ? argv[3] : "../../nao/home/"));

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

    CameraMatrix camMat;
    camMat.fc = fc;
    camMat.fc.x() *= imSize.x();
    camMat.fc.y() *= imSize.y();
    camMat.cc = cc;
    camMat.cc.x() *= imSize.x();
    camMat.cc.y() *= imSize.y();
    camMat.fov = fov;

    const ObservationModelConfig cfg = {imSize, fc, cc, fov, {SENSOR_NAME::BOTTOM_CAMERA}, 1000, 25, 0.05};

    // weirdPointTest(imSize, fc, cc, fov);

    weirdPointTest2(cfg);
    std::cout << "end test Cam Obs" << std::endl;

    weirdPointTest2ObsProvider(cfg);

    std::cout << "end test obs provider" << std::endl;
    // std::string line;

    // format: jointNo joint1 jont2 ...
    // JointAndPoints jointVals;

    // size_t iters = 0;
    // while (getline(inFile, line) && iters < 20)
    // {
    //     std::istringstream ss(line);
    //     int j = -1;
    //     std::vector<float> joints;

    //     ss >> j;
    //     if (j >= 0 && ss.good())
    //     {
    //         float jVal = 0;
    //         for(int i=0; i<static_cast<int>(JOINTS::JOINT::JOINTS_MAX; i++){
    //             ss >> jVal;
    //             joints.push_back(jVal);
    //         }
    //         jointVals.emplace_back(j, joints);
    //         weirdPointTest(jointVals, imSize,fc,cc,fov);
    //     }
    //     iters++;
    // }

    // // 'x' is vector of length 'n' containing the initial values for the parameters.
    // // The parameters 'x' are also referred to as the 'inputs' in the context of LM optimization.
    // // The LM optimization inputs should not be confused with the x input values.
    // Eigen::Vector3f x(0, 0, 0);
    // // x(0) = 0.0; // initial value for 'a'
    // // x(1) = 0.0; // initial value for 'b'
    // // x(2) = 0.0; // initial value for 'c'

    // auto start = std::chrono::steady_clock::now();

    // auto end = std::chrono::steady_clock::now();

    // std::cout << "Elapsed time in nanoseconds : "
    //           << std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count()
    //           << " ns" << std::endl;

    // std::cout << "Elapsed time in microseconds : "
    //           << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
    //           << " µs" << std::endl;

    // std::cout << "Elapsed time in milliseconds : "
    //           << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
    //           << " ms" << std::endl;

    // std::cout << "Elapsed time in seconds : "
    //           << std::chrono::duration_cast<std::chrono::seconds>(end - start).count()
    //           << " sec" << std::endl;

    // // std::cout << "LM optimization status: " << status << std::endl;

    // //
    // // Results
    // // The 'x' vector also contains the results of the optimization.
    // //
    // std::cout << "Optimization results " << status << std::endl;
    // std::cout << "\ta: " << x(0) << std::endl;
    // std::cout << "\tb: " << x(1) << std::endl;
    // std::cout << "\tc: " << x(2) << std::endl;

    return 0;
}