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

/**
 * This test is for evaluation. It'll construct the ground grid similar to the observation model and use it to perform calibration.
 * This program also will include joint errors into the modelling.
 *
 * Then it'll calibrate and see if the calibration can compensate the errors.
 */


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

    // std::string supportFootName((argc > 2 ? argv[2] : "d"));
    std::string confRoot((argc > 1 ? argv[1] : "../../nao/home/"));

    // SUPPORT_FOOT supportFoot = SUPPORT_FOOT::SF_DOUBLE;
    // if (supportFootName.compare("l") == 0)
    // {
    //     supportFoot = SUPPORT_FOOT::SF_LEFT;
    //     std::cout << "Left Support foot." << std::endl;
    // }
    // else if (supportFootName.compare("r") == 0)
    // {
    //     supportFoot = SUPPORT_FOOT::SF_RIGHT;
    //     std::cout << "Right Support foot." << std::endl;
    // }
    // else
    // {
    //     std::cout << "Going for double foot." << std::endl;
    // }

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