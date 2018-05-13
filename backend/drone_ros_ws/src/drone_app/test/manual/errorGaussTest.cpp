#include <iostream>
#include "include/csv.h"

#include "common/Pose.hpp"

#include "detector/gazebo/GaussErrorGenerator.hpp"

GaussErrorGenerator* generator;

double mean;
double std_dev;

void printHelp()
{
    std::cerr << "errorGaussTest <csv> <mean> <standard deviation>" << std::endl;
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
    if(argc < 4)
    {
        printHelp();
        return EXIT_FAILURE;
    }

    mean = std::stod(argv[2]);
    std_dev = std::stod(argv[3]);

    generator = new GaussErrorGenerator(mean, std_dev);

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
