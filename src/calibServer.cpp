////
////  Hello World server in C++
////  Binds REP socket to tcp://*:5555
////  Expects "Hello" from client, replies with "World"
////
//#include <zmq.hpp>
//#include <string>
//#include <iostream>
//#ifndef _WIN32
//#include <unistd.h>
//#else
//#include <windows.h>

//#define sleep(n)    Sleep(n)
//#endif

//int main () {
//    //  Prepare our context and socket
//    zmq::context_t context (1);
//    zmq::socket_t socket (context, ZMQ_REP);
//    socket.bind ("tcp://*:5555");

//    while (true) {
//        zmq::message_t request;

//        //  Wait for next request from client
//        socket.recv (&request);
//        std::cout << "Received Hello" << std::endl;

//        //  Do some 'work'
//        sleep(1);

//        //  Send reply back to client
//        zmq::message_t reply (5);
//        memcpy (reply.data (), "World", 5);
//        socket.send (reply);
//    }
//    return 0;
//}

//void calibEval(){


//    /*
//     * Mini eval of cost fcn
//     */

//    auto calibrator = JointCalibrator(frameCaptures, naoJointSensorModel);

//    rawPoseT calVec(JOINTS::JOINT::JOINTS_MAX, 0.0);

//    Eigen::VectorXf inducedErrorEig = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(inducedErrorStdVec.data(), inducedErrorStdVec.size());

//    Eigen::VectorXf errorVec(calibrator.values());
//    Eigen::VectorXf finalErrorVec(calibrator.values());

//    std::cout<< "Compensated Calib state: " << calibrator(inducedErrorEig, errorVec) << " cost: " << errorVec.norm() << std::endl;

//    std::cout<< "Without Calib state: " << calibrator(Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(calVec.data(), calVec.size()), errorVec) << " cost: " << errorVec.norm() << std::endl;



//    Eigen::NumericalDiff<JointCalibrator> functorDiff(calibrator);
//    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<JointCalibrator>, float> lm(functorDiff);

//    Eigen::VectorXf calibratedParams;
//    calibratedParams.resize(JOINTS::JOINT::JOINTS_MAX);

//    int status = lm.minimize(calibratedParams);

//    std::cout<< "Calib status: "<< status <<
//                "\nInduced Error and calib diff: \n" << ((inducedErrorEig - calibratedParams)/ TO_RAD).transpose() <<
//                "\nCalibration values:   \n" << (calibratedParams / TO_RAD).transpose() <<std::endl;

//    std::cout<< "Calibrated status:: " << calibrator(calibratedParams, finalErrorVec) << " cost: " << errorVec.norm() << std::endl;

//    std::cout<< "Dumping residuals" << std::endl;

//    std::pair<Eigen::VectorXf &, std::string> originalResidualDump(errorVec, inFileName + ".originalResidual");
//    std::pair<Eigen::VectorXf &, std::string> calibratedResidualDump(finalErrorVec, inFileName + ".calibratedResidual");


//    for(auto elem : {originalResidualDump, calibratedResidualDump}){
//        std::cout<< "Start dump to : " << elem.second << std::endl;
//        std::fstream file(elem.second, std::ios::out);
//        if(file){
//            auto & residualVec = elem.first;
//            for(long i = 0; i < residualVec.size(); ++i){
//                if(i % 2 == 0){ // even number
//                    file << residualVec(i);
//                }else{ // odd  -> newline
//                    file << "," << residualVec(i) << "\n";
//                }
//            }
//        }
//    }

//}
