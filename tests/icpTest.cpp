#include <iostream>
// #include <iomanip>
#include <fstream>
#include <sstream>
#include <cmath>
#include <chrono>

#include <Eigen/Eigen>
#include <Eigen/Geometry>

#ifndef TO_RAD
#define TO_RAD M_PI / 180
#endif

using namespace Eigen;

#include "Solvers.hpp"

int main(int argc, char *argv[])
{
    //
    // Goal
    //
    // Given a non-linear equation: f(x) = a(x^2) + b(x) + c
    // and 'm' data points (x1, f(x1)), (x2, f(x2)), ..., (xm, f(xm))
    // our goal is to estimate 'n' parameters (3 in this case: a, b, c)
    // using LM optimization.
    //

    //
    // Read values from file.
    // Each row has two numbers, for example: 5.50 223.70
    // The first number is the input value (5.50) i.e. the value of 'x'.
    // The second number is the observed output value (223.70),
    // i.e. the measured value of 'f(x)'.
    //
    std::ifstream inFile;
    if (argc > 1)
    {
        inFile.open(argv[1]);
    }
    if (!inFile)
    {
        std::cout << "Unable to read file." << std::endl;
        return -1;
    }

    std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f>> values;
    /**
	 * Dummy transform
	 */
    std::cout << (20 * TO_RAD) << std::endl;
    Eigen::Rotation2D<float> rot2(60 * TO_RAD);
    Eigen::Translation<float, 2> trans(500, -4500);

    std::string line;

    size_t iters = 0;
    while (getline(inFile, line) && iters < 20)
    {
        std::istringstream ss(line);
        Eigen::Vector2f orig;
        Eigen::Vector2f transformed;
        ss >> orig.x() >> orig.y();

        transformed = trans * rot2 * orig;
        values.emplace_back(orig, transformed);

        iters++;
    }

    // 'x' is vector of length 'n' containing the initial values for the parameters.
    // The parameters 'x' are also referred to as the 'inputs' in the context of LM optimization.
    // The LM optimization inputs should not be confused with the x input values.
    Eigen::Vector3f x(0, 0, 0);
    // x(0) = 0.0; // initial value for 'a'
    // x(1) = 0.0; // initial value for 'b'
    // x(2) = 0.0; // initial value for 'c'

    auto start = std::chrono::steady_clock::now();
    int status = ObservationSolvers::get2dPose(values, x);
    auto end = std::chrono::steady_clock::now();

    std::cout << "Elapsed time in nanoseconds : "
              << std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count()
              << " ns" << std::endl;

    std::cout << "Elapsed time in microseconds : "
              << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
              << " µs" << std::endl;

    std::cout << "Elapsed time in milliseconds : "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
              << " ms" << std::endl;

    std::cout << "Elapsed time in seconds : "
              << std::chrono::duration_cast<std::chrono::seconds>(end - start).count()
              << " sec" << std::endl;

    // std::cout << "LM optimization status: " << status << std::endl;

    //
    // Results
    // The 'x' vector also contains the results of the optimization.
    //
    std::cout << "Optimization results " << status << std::endl;
    std::cout << "\ta: " << x(0) << std::endl;
    std::cout << "\tb: " << x(1) << std::endl;
    std::cout << "\tc: " << x(2) << std::endl;

    return 0;
}