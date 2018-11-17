#include <iostream>
// #include <iomanip>
#include <fstream>
#include <sstream>
#include <cmath>
#include <chrono>
#include <thread>
#include <array>

#include <Data/CameraMatrix.hpp>
#include <Modules/NaoProvider.h>
#include <Modules/Configuration/Configuration.h>
#include <Modules/Configuration/UnixSocketConfig.hpp>
#include <Modules/Poses.h>

#include <Tools/Storage/Image.hpp>
#include <Tools/Storage/Image422.hpp>
#include <Tools/Kinematics/KinematicMatrix.h>
#include <Hardware/RobotInterface.hpp>

#include "constants.hpp"
#include "TUHHMin.hpp"
#include "NaoPoseInfo.hpp"
#include "NaoStability.hpp"
#include "NaoTorsoPose.hpp"
#include "NaoJointAndSensorModel.hpp"

#define DEBUG_CAM_OBS 1
#include "CameraObservationModel.hpp"
#include "ObservationSensitivityProvider.hpp"

#define DO_COMMIT 1
#define WRITE_PARALLEL 1
#define DEBUG_FILE_COMMIT 1
#define DEBUG_IN_THREAD 1
#define ENABLE_PROGRESS 1

#include "utils.hpp"

typedef std::vector<std::pair<int, std::vector<float>>> JointAndPoints;

const unsigned int MAX_THREADS = std::thread::hardware_concurrency();
const unsigned int MAX_POSES_TO_CALIB = 20;

int main(int argc, char *argv[])
{
    std::string inFileName((argc > 1 ? argv[1] : "out"));
    std::string confRoot((argc > 2 ? argv[2] : "../../nao/home/"));

    size_t usableThreads = 0;

    std::fstream inFile(inFileName);
//    std::vector<std::fstream> inputPoseAndJointStreams;
//    for (size_t i = 0; i < MAX_THREADS; i++)
//    {
//        std::string fileName = inFileName + "_" + constants::ExtractedSensitivitiesFileName + "_" + std::to_string(i) + ".txt";
//        if (std::ifstream(fileName))
//        {
//            inputPoseAndJointStreams.emplace_back(fileName, std::ios::in);
//            usableThreads++;
//        }
//        else
//        {
//            break;
//        }
//    }

    /*
     * Initializing model
     */
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

    auto naoJointSensorModel = NaoJointAndSensorModel(imSize,fc, cc,fov, cfg.maxGridPointsPerSide, cfg.gridSpacing);

    // TODO.. actual calib values and reporting ;)
    naoJointSensorModel.setCalibValues(rawPoseT(JOINTS::JOINT::JOINTS_MAX));

    NaoPoseAndRawAngles<float> poseAndAngles;

    auto camName = Camera::BOTTOM;

    // TODO multithread later...
//    for(auto & iStreamRef : inputPoseAndJointStreams){
    std::vector<std::pair<Vector2f, Vector2f>> allFrameCorrespondances;

    for(size_t i=0; i < MAX_POSES_TO_CALIB && utils::JointsAndPosesStream::getNextPoseAndRawAngles(inFile, poseAndAngles); ++i){

        naoJointSensorModel.setPose(poseAndAngles.angles, poseAndAngles.pose.supportFoot);
        bool success = false;
        // will be relative to support foot, easier to manage
        auto groundGrid = naoJointSensorModel.getGroundGrid(camName, success);
        if(success){
            // world points, pixel points.
            // TODO make world points handle 3d also
            auto correspondances = NaoSensorDataProvider::getFilteredCorrespondancePairs(groundGrid, naoJointSensorModel.robotToPixelMulti(camName, groundGrid));

            allFrameCorrespondances.insert(allFrameCorrespondances.end(),
                                       std::make_move_iterator(correspondances.begin()),
                                       std::make_move_iterator(correspondances.end()));

        }else{
            std::cerr << "Obtaining ground grid failed" << std::endl;
        }
    }

//    }

    /*
     * Now we need a cost function!!
     *
     */

    auto costFunc = [](){

    };

    return 0;
}
