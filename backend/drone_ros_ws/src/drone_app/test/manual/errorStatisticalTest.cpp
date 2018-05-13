#include <iostream>
#include "include/csv.h"

#include "common/Pose.hpp"

#include "detector/gazebo/StatisticalErrorGenerator.hpp"

StatisticalErrorGenerator* generator;

int    rate;
double min;
double max;

void printHelp()
{
    std::cerr << "errorUniformTest <csv> <frequency> <min> <max>" << std::endl;
}

void handle(Pose p)
{
    Pose err = generator->generatePose();

    Pose sum = p + err;

    std::cout << p.posX << ", " << p.posY << ", " << p.posZ << ", "
              << err.posX << ", " << err.posY << ", " << err.posZ << ", "
              << sum.posX << ", " << sum.posY << ", " << sum.posZ << ", "
              << std::endl;
}

int main(int argc, char** argv)
{
    if(argc < 5)
    {
        printHelp();
        return EXIT_FAILURE;
    }

    rate = std::stoi(argv[2]);
    min  = std::stod(argv[3]);
    max  = std::stod(argv[4]);

    generator = new StatisticalErrorGenerator(rate, min, max);

    // Header

    std::cout << "posX, posY, posZ, errX, errY, errZ, sumX, sumY, sumZ," << std::endl;

    // CSV

    io::CSVReader<6> csv(argv[1]);
    csv.read_header(io::ignore_extra_column, "posX", "posY", "posZ", "rotX", "rotY", "rotZ");

    Pose p;

    while(csv.read_row(p.posX, p.posY, p.posZ, p.rotX, p.rotY, p.rotZ))
    {
        handle(p);

        p.posX =
        p.posY =
        p.posZ =
        p.rotX =
        p.rotY =
        p.rotZ = 0;
    }

    delete generator;

    return EXIT_SUCCESS;
}
