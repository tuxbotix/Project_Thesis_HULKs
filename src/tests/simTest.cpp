#include <iostream>
// #include <iomanip>
#include <fstream>
#include <sstream>
#include <cmath>
#include <chrono>
#include <thread>
#include <array>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <unsupported/Eigen/NonLinearOptimization>

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
// first is ground point
typedef std::vector<std::pair<Vector2f, Vector2f>> CorrespondanceList;
typedef std::vector<std::pair<Vector3f, Vector2f>> Correspondance3D2DList;

//struct JointCalibCapture{
//    NaoPose<float> pose;
//    Correspondance3D2DList capturedCorrespondances;
//};

struct JointCalibCaptureEvalT{
    CorrespondanceList capturedCorrespondances;
    rawPoseT pose;
    SUPPORT_FOOT sf;
    Camera camName;

    JointCalibCaptureEvalT(const CorrespondanceList & correspondances, rawPoseT & pose, Camera camName, SUPPORT_FOOT sf){
        capturedCorrespondances = correspondances;
        this->pose = pose;
        this->camName = camName;
        this->sf = sf;
    }
};

typedef std::vector<JointCalibCaptureEvalT> CaptureList;

const unsigned int MAX_THREADS = std::thread::hardware_concurrency();
const unsigned int MAX_POSES_TO_CALIB = 15;

///**
// * Extracted from https://github.com/daviddoria/Examples/blob/master/c%2B%2B/Eigen/LevenbergMarquardt/CurveFitting.cpp
// * This definition structure is needed to use NumericalDiff module
// */

template <typename _Scalar = float, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
struct Functor
{
  typedef _Scalar Scalar;
  enum
  {
    InputsAtCompileTime = NX,
    ValuesAtCompileTime = NY
  };
  typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
  typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
  typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

  int n_inputs, m_values;

  Functor() : n_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
  Functor(int inputs, int values) : n_inputs(inputs), m_values(values) {}

  int inputs() const { return n_inputs; }
  int values() const { return m_values; }
};

struct JointCalibrator : Functor<float>{

    const CaptureList captureList;
    const size_t captureDataSetSize;
    const NaoJointAndSensorModel naoModel;

    int operator()(const Eigen::VectorXf &calibrationVals, Eigen::VectorXf &errorVec) const {
        auto tempModel = NaoJointAndSensorModel(naoModel);

        rawPoseT calibrationValsStdVec;
        calibrationValsStdVec.resize(calibrationVals.size());
        Eigen::VectorXf::Map(&calibrationValsStdVec[0], calibrationVals.size()) = calibrationVals;


        tempModel.setCalibValues(calibrationValsStdVec);
        long totalErrVecSize = 0;
//        error.resize(capture.);
        rawPoseListT errorVecVec(captureList.size());

        for(size_t capIdx = 0; capIdx < captureList.size(); capIdx++){
            const auto & capture = captureList[capIdx];
            auto & errorAtCap = errorVecVec[capIdx];
            const auto & camName = capture.camName;

            tempModel.setPoseCamUpdateOnly(capture.pose, capture.sf, capture.camName);
            for(const auto & correspondance : capture.capturedCorrespondances){
                Vector2f error;
                if(tempModel.robotToPixelFlt(camName, correspondance.first, error)){
                    error -= correspondance.second;

                    errorAtCap.push_back(error.x());
                    errorAtCap.push_back(error.y());

                    totalErrVecSize++;
                }
            }
        }
//            auto correspondances = NaoSensorDataProvider::getFilteredCorrespondancePairs(groundGrid, naoJointSensorModel.robotToPixelMulti(camName, groundGrid));
        int iter = 0;
        for(size_t i = 0; i< errorVecVec.size(); ++i){
            auto & errorAtCap = errorVecVec[i];
            for(size_t j = 0; j< errorAtCap.size(); ++j, ++iter){
                errorVec(iter) = errorAtCap[j];
            }
        }
        return 0;
    }

    JointCalibrator(CaptureList & captures, NaoJointAndSensorModel & model):
        captureList(captures),
        // This nasty chunk get total element count via a lambda and return to the int.
        captureDataSetSize([=]()-> size_t{
            size_t temp = 0;
            for(auto & elem : captures){
                temp += elem.capturedCorrespondances.size() * 2;
            }
            return temp;
        }()),
        naoModel(model)
    {

    }

    // Means the parameter count = 26~ in our case
    int inputs() const { return JOINTS::JOINT::JOINTS_MAX;  }
    // size of sample/ correspondance set
    size_t values() const { return captureDataSetSize;}
};

int main(int argc, char *argv[])
{
    std::string inFileName((argc > 1 ? argv[1] : "out"));
    std::string confRoot((argc > 2 ? argv[2] : "../../nao/home/"));

//    size_t usableThreads = 0;

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
    auto inducedErrorStdVec = rawPoseT(JOINTS::JOINT::JOINTS_MAX);
    inducedErrorStdVec[JOINTS::JOINT::L_KNEE_PITCH] = 5 * TO_RAD;
    inducedErrorStdVec[JOINTS::JOINT::L_HIP_PITCH] = -6 * TO_RAD;

    naoJointSensorModel.setCalibValues(inducedErrorStdVec);

    NaoPoseAndRawAngles<float> poseAndAngles;

    auto camNames = {Camera::TOP, Camera::BOTTOM};

    // TODO multithread later...
//    for(auto & iStreamRef : inputPoseAndJointStreams){
    CaptureList frameCaptures;

    for(size_t i=0; i < MAX_POSES_TO_CALIB && utils::JointsAndPosesStream::getNextPoseAndRawAngles(inFile, poseAndAngles); ++i){

        naoJointSensorModel.setPose(poseAndAngles.angles, poseAndAngles.pose.supportFoot);
        for(auto & camName : camNames){
            bool success = false;
            // will be relative to support foot, easier to manage
            auto groundGrid = naoJointSensorModel.getGroundGrid(camName, success);
            if(success){
                // world points, pixel points.
                // TODO make world points handle 3d also
                auto correspondances = NaoSensorDataProvider::getFilteredCorrespondancePairs(groundGrid, naoJointSensorModel.robotToPixelMulti(camName, groundGrid));

                frameCaptures.emplace_back(correspondances, poseAndAngles.angles, camName, poseAndAngles.pose.supportFoot);
            }else{
                std::cerr << "Obtaining ground grid failed" << std::endl;
            }
        }
    }


    /*
     * Mini eval of cost fcn
     */

    auto calibrator = JointCalibrator(frameCaptures, naoJointSensorModel);

    rawPoseT calVec(JOINTS::JOINT::JOINTS_MAX, 0.0);

    Eigen::VectorXf inducedErrorEig = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(inducedErrorStdVec.data(), inducedErrorStdVec.size());

    Eigen::VectorXf errorVec(calibrator.values());
    Eigen::VectorXf finalErrorVec(calibrator.values());

    std::cout<< "Compensated Calib state: " << calibrator(inducedErrorEig, errorVec) << " cost: " << errorVec.norm() << std::endl;

    std::cout<< "Without Calib state: " << calibrator(Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(calVec.data(), calVec.size()), errorVec) << " cost: " << errorVec.norm() << std::endl;



    Eigen::NumericalDiff<JointCalibrator> functorDiff(calibrator);
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<JointCalibrator>, float> lm(functorDiff);

    Eigen::VectorXf calibratedParams;
    calibratedParams.resize(JOINTS::JOINT::JOINTS_MAX);

    int status = lm.minimize(calibratedParams);

    std::cout<< "Calib status: "<< status <<
                "\nInduced Error and calib diff: \n" << ((inducedErrorEig - calibratedParams)/ TO_RAD).transpose() <<
                "\nCalibration values:   \n" << (calibratedParams / TO_RAD).transpose() <<std::endl;

    std::cout<< "Calibrated status:: " << calibrator(calibratedParams, finalErrorVec) << " cost: " << errorVec.norm() << std::endl;

    std::cout<< "Dumping residuals" << std::endl;

    std::pair<Eigen::VectorXf &, std::string> originalResidualDump(errorVec, inFileName + ".originalResidual");
    std::pair<Eigen::VectorXf &, std::string> calibratedResidualDump(finalErrorVec, inFileName + ".calibratedResidual");


    for(auto elem : {originalResidualDump, calibratedResidualDump}){
        std::cout<< "Start dump to : " << elem.second << std::endl;
        std::fstream file(elem.second, std::ios::out);
        if(file){
            auto & residualVec = elem.first;
            for(long i = 0; i < residualVec.size(); ++i){
                if(i % 2 == 0){ // even number
                    file << residualVec(i);
                }else{ // odd  -> newline
                    file << "," << residualVec(i) << "\n";
                }
            }
        }
    }

    return 0;
}
