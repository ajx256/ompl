/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2024, Andrew Mitchell
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Andrew Mitchell nor the names of any
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Andrew Mitchell */

#define PRINT_DUBINS_3D_RESULTS
#define SAMPLE_AND_DUMP_PATH

#define BOOST_TEST_MODULE "Dubins3DStateSpace"
#include <boost/test/unit_test.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/algorithm/string.hpp> 

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/spaces/Dubins3DStateSpace.h"

#include <iostream>
#include <fstream>
#include <chrono>

using namespace ompl;

void runTest(std::string testName, double desired, 
             double startX, double startY, double startZ, double startPitch, double startYaw,
             double endX, double endY, double endZ, double endPitch, double endYaw, double diffPercent = 0.1)
{
    auto stateSpace(std::make_shared<base::Dubins3DStateSpace>(40.0, 
        base::Dubins3DStateSpace::degToRad(-15), 
        base::Dubins3DStateSpace::degToRad(20)));

    auto start = stateSpace->allocState()->as<base::Dubins3DStateSpace::StateType>();
    start->setXYZ(startX, startY, startZ);
    start->setRPY(0.0, startPitch, startYaw);

    auto end = stateSpace->allocState()->as<base::Dubins3DStateSpace::StateType>();
    end->setXYZ(endX, endY, endZ);
    end->setRPY(0.0, endPitch, endYaw);

    auto tStart = std::chrono::high_resolution_clock::now();
    base::Dubins3DStateSpace::Dubins3DPath path = stateSpace->dubins(start, end);
    auto tEnd = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> durms = tEnd - tStart;

    stateSpace->freeState(start);
    stateSpace->freeState(end);

#ifdef PRINT_DUBINS_3D_RESULTS
    printf("%s L: %.2f T: %5.3f ms H: type: %s rho: %.2f t: %.2f p: %.2f q: %.2f V: type: %s rho: %.2f t: %.2f p: %.2f q: %.2f\n",
            testName.c_str(), path.length(), durms.count(), 
            stateSpace->dubinsPathTypeToString(path.horizontalPath_).c_str(), path.rhoH_, path.horizontalPath_.length_[0], 
            path.horizontalPath_.length_[1], path.horizontalPath_.length_[2],
            stateSpace->dubinsPathTypeToString(path.verticalPath_).c_str(), path.rhoV_, path.verticalPath_.length_[0], 
            path.verticalPath_.length_[1], path.verticalPath_.length_[2]);
#endif

#ifdef SAMPLE_AND_DUMP_PATH
    boost::algorithm::erase_all(testName, " ");
    std::ofstream pathFile(testName + ".txt", std::ofstream::trunc);
    std::vector<base::State*> samples = stateSpace->sampleStates(500, path);
    for (base::State* sample : samples)
    {
        std::string sampleStr = sample->as<base::Dubins3DStateSpace::StateType>()->toString();
        // Remove the opening square bracket
        sampleStr = sampleStr.erase(0, 1);
        // Remove the closing bracket
        sampleStr = sampleStr.erase(sampleStr.length() - 1, 1);
        // Remove all commas
        boost::algorithm::erase_all(sampleStr, ",");
        pathFile << sampleStr << std::endl;
        stateSpace->freeState(sample);
    }
    pathFile.close();
#endif

    BOOST_CHECK_CLOSE(desired, path.length(), diffPercent);
}

BOOST_AUTO_TEST_CASE(Long_1)
{
    double desired = 446.04;
    std::string testName = "Long 1";
    double startX = 200;
    double startY = 500;
    double startZ = 200;
    double startPitch = base::Dubins3DStateSpace::degToRad(-5);
    double startYaw = base::Dubins3DStateSpace::degToRad(180);
    double endX = 500;
    double endY = 350;
    double endZ = 100;
    double endPitch = base::Dubins3DStateSpace::degToRad(-5);
    double endYaw = base::Dubins3DStateSpace::degToRad(0);

    runTest(testName, desired, startX, startY, startZ, startPitch, startYaw, endX, endY, endZ, endPitch, endYaw);
}

BOOST_AUTO_TEST_CASE(Long_2)
{
    double desired = 638.45;
    std::string testName = "Long 2";
    double startX = 100;
    double startY = -400;
    double startZ = 100;
    double startPitch = base::Dubins3DStateSpace::degToRad(0);
    double startYaw = base::Dubins3DStateSpace::degToRad(30);
    double endX = 500;
    double endY = -700;
    double endZ = 0;
    double endPitch = base::Dubins3DStateSpace::degToRad(0);
    double endYaw = base::Dubins3DStateSpace::degToRad(150);

    runTest(testName, desired, startX, startY, startZ, startPitch, startYaw, endX, endY, endZ, endPitch, endYaw);
}

BOOST_AUTO_TEST_CASE(Long_3)
{
    double desired = 1068.34;
    std::string testName = "Long 3";
    double startX = -200;
    double startY = 200;
    double startZ = 250;
    double startPitch = base::Dubins3DStateSpace::degToRad(15);
    double startYaw = base::Dubins3DStateSpace::degToRad(240);
    double endX = 500;
    double endY = 800;
    double endZ = 0;
    double endPitch = base::Dubins3DStateSpace::degToRad(15);
    double endYaw = base::Dubins3DStateSpace::degToRad(45);

    runTest(testName, desired, startX, startY, startZ, startPitch, startYaw, endX, endY, endZ, endPitch, endYaw);
}

BOOST_AUTO_TEST_CASE(Long_4)
{
    double desired = 1788.80;
    std::string testName = "Long 4";
    double startX = -300;
    double startY = 1200;
    double startZ = 350;
    double startPitch = base::Dubins3DStateSpace::degToRad(0);
    double startYaw = base::Dubins3DStateSpace::degToRad(160);
    double endX = 1000;
    double endY = 200;
    double endZ = 0;
    double endPitch = base::Dubins3DStateSpace::degToRad(0);
    double endYaw = base::Dubins3DStateSpace::degToRad(30);

    runTest(testName, desired, startX, startY, startZ, startPitch, startYaw, endX, endY, endZ, endPitch, endYaw);
}

BOOST_AUTO_TEST_CASE(Long_5)
{
    double desired = 2214.54;
    std::string testName = "Long 5";
    double startX = -500;
    double startY = -300;
    double startZ = 600;
    double startPitch = base::Dubins3DStateSpace::degToRad(10);
    double startYaw = base::Dubins3DStateSpace::degToRad(150);
    double endX = 1200;
    double endY = 900;
    double endZ = 100;
    double endPitch = base::Dubins3DStateSpace::degToRad(10);
    double endYaw = base::Dubins3DStateSpace::degToRad(300);

    runTest(testName, desired, startX, startY, startZ, startPitch, startYaw, endX, endY, endZ, endPitch, endYaw);
}

BOOST_AUTO_TEST_CASE(Short_1)
{
    double desired = 580.79;
    std::string testName = "Short 1";
    double startX = 120;
    double startY = -30;
    double startZ = 250;
    double startPitch = base::Dubins3DStateSpace::degToRad(-10);
    double startYaw = base::Dubins3DStateSpace::degToRad(100);
    double endX = 220;
    double endY = 150;
    double endZ = 100;
    double endPitch = base::Dubins3DStateSpace::degToRad(-10);
    double endYaw = base::Dubins3DStateSpace::degToRad(300);

    runTest(testName, desired, startX, startY, startZ, startPitch, startYaw, endX, endY, endZ, endPitch, endYaw);
}

BOOST_AUTO_TEST_CASE(Short_2)
{
    double desired = 668.17;
    std::string testName = "Short 2";
    double startX = 380;
    double startY = 230;
    double startZ = 200;
    double startPitch = base::Dubins3DStateSpace::degToRad(0);
    double startYaw = base::Dubins3DStateSpace::degToRad(30);
    double endX = 280;
    double endY = 150;
    double endZ = 30;
    double endPitch = base::Dubins3DStateSpace::degToRad(0);
    double endYaw = base::Dubins3DStateSpace::degToRad(200);

    runTest(testName, desired, startX, startY, startZ, startPitch, startYaw, endX, endY, endZ, endPitch, endYaw);
}

BOOST_AUTO_TEST_CASE(Short_3)
{
    double desired = 976.79;
    std::string testName = "Short 3";
    double startX = -80;
    double startY = 10;
    double startZ = 250;
    double startPitch = base::Dubins3DStateSpace::degToRad(0);
    double startYaw = base::Dubins3DStateSpace::degToRad(20);
    double endX = 50;
    double endY = 70;
    double endZ = 0;
    double endPitch = base::Dubins3DStateSpace::degToRad(0);
    double endYaw = base::Dubins3DStateSpace::degToRad(240);

    runTest(testName, desired, startX, startY, startZ, startPitch, startYaw, endX, endY, endZ, endPitch, endYaw);
}

BOOST_AUTO_TEST_CASE(Short_4)
{
    double desired = 1169.80;
    std::string testName = "Short 4";
    double startX = 400;
    double startY = -250;
    double startZ = 600;
    double startPitch = base::Dubins3DStateSpace::degToRad(0);
    double startYaw = base::Dubins3DStateSpace::degToRad(350);
    double endX = 600;
    double endY = -150;
    double endZ = 300;
    double endPitch = base::Dubins3DStateSpace::degToRad(0);
    double endYaw = base::Dubins3DStateSpace::degToRad(150);

    runTest(testName, desired, startX, startY, startZ, startPitch, startYaw, endX, endY, endZ, endPitch, endYaw);
}

BOOST_AUTO_TEST_CASE(Short_5)
{
    double desired = 1362.91;
    std::string testName = "Short 5";
    double startX = -200;
    double startY = -200;
    double startZ = 450;
    double startPitch = base::Dubins3DStateSpace::degToRad(0);
    double startYaw = base::Dubins3DStateSpace::degToRad(340);
    double endX = -300;
    double endY = -80;
    double endZ = 100;
    double endPitch = base::Dubins3DStateSpace::degToRad(0);
    double endYaw = base::Dubins3DStateSpace::degToRad(100);

    runTest(testName, desired, startX, startY, startZ, startPitch, startYaw, endX, endY, endZ, endPitch, endYaw);
}

BOOST_AUTO_TEST_CASE(Beard_1)
{
    double desired = 976.89;
    std::string testName = "Beard 1";
    double startX = 0;
    double startY = 0;
    double startZ = 350;
    double startPitch = base::Dubins3DStateSpace::degToRad(0);
    double startYaw = base::Dubins3DStateSpace::degToRad(180) + M_PI_2;
    double endX = -100;
    double endY = 100;
    double endZ = 100;
    double endPitch = base::Dubins3DStateSpace::degToRad(0);
    double endYaw = base::Dubins3DStateSpace::degToRad(180) + M_PI_2;

    runTest(testName, desired, startX, startY, startZ, startPitch, startYaw, endX, endY, endZ, endPitch, endYaw);
}

BOOST_AUTO_TEST_CASE(Beard_2)
{
    double desired = 330.88;
    std::string testName = "Beard 2";
    double startX = 0;
    double startY = 0;
    double startZ = 100;
    double startPitch = base::Dubins3DStateSpace::degToRad(0);
    double startYaw = base::Dubins3DStateSpace::degToRad(70) + M_PI_2;
    double endX = 100;
    double endY = 100;
    double endZ = 125;
    double endPitch = base::Dubins3DStateSpace::degToRad(0);
    double endYaw = base::Dubins3DStateSpace::degToRad(70) + M_PI_2;

    runTest(testName, desired, startX, startY, startZ, startPitch, startYaw, endX, endY, endZ, endPitch, endYaw);
}

BOOST_AUTO_TEST_CASE(Beard_3)
{
    double desired = 371.87;
    std::string testName = "Beard 3";
    double startX = 0;
    double startY = 0;
    double startZ = 100;
    double startPitch = base::Dubins3DStateSpace::degToRad(0);
    double startYaw = base::Dubins3DStateSpace::degToRad(0) + M_PI_2;
    double endX = 200;
    double endY = 0;
    double endZ = 200;
    double endPitch = base::Dubins3DStateSpace::degToRad(0);
    double endYaw = base::Dubins3DStateSpace::degToRad(270) - M_PI_2;

    runTest(testName, desired, startX, startY, startZ, startPitch, startYaw, endX, endY, endZ, endPitch, endYaw);
}

BOOST_AUTO_TEST_CASE(Others_1)
{
    double desired = 1201.38;
    std::string testName = "Others 1";
    double startX = 500;
    double startY = 100;
    double startZ = 300;
    double startPitch = base::Dubins3DStateSpace::degToRad(15);
    double startYaw = base::Dubins3DStateSpace::degToRad(240);
    double endX = -100;
    double endY = 400;
    double endZ = 0;
    double endPitch = base::Dubins3DStateSpace::degToRad(15);
    double endYaw = base::Dubins3DStateSpace::degToRad(45);

    runTest(testName, desired, startX, startY, startZ, startPitch, startYaw, endX, endY, endZ, endPitch, endYaw);
}

BOOST_AUTO_TEST_CASE(Others_2)
{
    double desired = 251.41;
    std::string testName = "Others 2";
    double startX = 0;
    double startY = 0;
    double startZ = 0;
    double startPitch = base::Dubins3DStateSpace::degToRad(0);
    double startYaw = base::Dubins3DStateSpace::degToRad(0);
    double endX = 0;
    double endY = 0;
    double endZ = 5;
    double endPitch = base::Dubins3DStateSpace::degToRad(0);
    double endYaw = base::Dubins3DStateSpace::degToRad(0);

    runTest(testName, desired, startX, startY, startZ, startPitch, startYaw, endX, endY, endZ, endPitch, endYaw);
}

BOOST_AUTO_TEST_CASE(Others_3)
{
    double desired = 251.67;
    std::string testName = "Others 3";
    double startX = 0;
    double startY = 0;
    double startZ = 0;
    double startPitch = base::Dubins3DStateSpace::degToRad(0);
    double startYaw = base::Dubins3DStateSpace::degToRad(0);
    double endX = 0;
    double endY = 0;
    double endZ = 10;
    double endPitch = base::Dubins3DStateSpace::degToRad(0);
    double endYaw = base::Dubins3DStateSpace::degToRad(0);

    runTest(testName, desired, startX, startY, startZ, startPitch, startYaw, endX, endY, endZ, endPitch, endYaw);
}

BOOST_AUTO_TEST_CASE(Others_4)
{
    double desired = 252.70;
    std::string testName = "Others 4";
    double startX = 0;
    double startY = 0;
    double startZ = 0;
    double startPitch = base::Dubins3DStateSpace::degToRad(0);
    double startYaw = base::Dubins3DStateSpace::degToRad(0);
    double endX = 0;
    double endY = 0;
    double endZ = 20;
    double endPitch = base::Dubins3DStateSpace::degToRad(0);
    double endYaw = base::Dubins3DStateSpace::degToRad(0);

    runTest(testName, desired, startX, startY, startZ, startPitch, startYaw, endX, endY, endZ, endPitch, endYaw);
}

BOOST_AUTO_TEST_CASE(Others_5)
{
    double desired = 254.39;
    std::string testName = "Others 5";
    double startX = 0;
    double startY = 0;
    double startZ = 0;
    double startPitch = base::Dubins3DStateSpace::degToRad(0);
    double startYaw = base::Dubins3DStateSpace::degToRad(0);
    double endX = 0;
    double endY = 0;
    double endZ = 30;
    double endPitch = base::Dubins3DStateSpace::degToRad(0);
    double endYaw = base::Dubins3DStateSpace::degToRad(0);

    runTest(testName, desired, startX, startY, startZ, startPitch, startYaw, endX, endY, endZ, endPitch, endYaw);
}

BOOST_AUTO_TEST_CASE(Others_6)
{
    double desired = 259.66;
    std::string testName = "Others 6";
    double startX = 0;
    double startY = 0;
    double startZ = 0;
    double startPitch = base::Dubins3DStateSpace::degToRad(0);
    double startYaw = base::Dubins3DStateSpace::degToRad(0);
    double endX = 0;
    double endY = 0;
    double endZ = 50;
    double endPitch = base::Dubins3DStateSpace::degToRad(0);
    double endYaw = base::Dubins3DStateSpace::degToRad(0);

    runTest(testName, desired, startX, startY, startZ, startPitch, startYaw, endX, endY, endZ, endPitch, endYaw);
}

BOOST_AUTO_TEST_CASE(Others_7)
{
    double desired = 309.08;
    std::string testName = "Others 7";
    double startX = 0;
    double startY = 0;
    double startZ = 0;
    double startPitch = base::Dubins3DStateSpace::degToRad(10);
    double startYaw = base::Dubins3DStateSpace::degToRad(0);
    double endX = 292.4;
    double endY = 0;
    double endZ = 100;
    double endPitch = base::Dubins3DStateSpace::degToRad(10);
    double endYaw = base::Dubins3DStateSpace::degToRad(0);

    runTest(testName, desired, startX, startY, startZ, startPitch, startYaw, endX, endY, endZ, endPitch, endYaw);
}

BOOST_AUTO_TEST_CASE(Others_8)
{
    double desired = 706.66;
    std::string testName = "Others 8";
    double startX = 0;
    double startY = 0;
    double startZ = 200;
    double startPitch = base::Dubins3DStateSpace::degToRad(0);
    double startYaw = base::Dubins3DStateSpace::degToRad(0);
    double endX = 0;
    double endY = 0;
    double endZ = 20;
    double endPitch = base::Dubins3DStateSpace::degToRad(0);
    double endYaw = base::Dubins3DStateSpace::degToRad(0);

    runTest(testName, desired, startX, startY, startZ, startPitch, startYaw, endX, endY, endZ, endPitch, endYaw);
}

BOOST_AUTO_TEST_CASE(Hota_1)
{
    double desired = 553.79;
    std::string testName = "Hota 1";
    double startX = 100;
    double startY = 10;
    double startZ = 200;
    double startPitch = base::Dubins3DStateSpace::degToRad(20);
    double startYaw = base::Dubins3DStateSpace::degToRad(180);
    double endX = 500;
    double endY = 40;
    double endZ = 100;
    double endPitch = base::Dubins3DStateSpace::degToRad(20);
    double endYaw = base::Dubins3DStateSpace::degToRad(0);

    runTest(testName, desired, startX, startY, startZ, startPitch, startYaw, endX, endY, endZ, endPitch, endYaw);
}

BOOST_AUTO_TEST_CASE(Hota_2)
{
    double desired = 254.19;
    std::string testName = "Hota 2";
    double startX = 100;
    double startY = 10;
    double startZ = 200;
    double startPitch = base::Dubins3DStateSpace::degToRad(20);
    double startYaw = base::Dubins3DStateSpace::degToRad(180);
    double endX = 120;
    double endY = 20;
    double endZ = 210;
    double endPitch = base::Dubins3DStateSpace::degToRad(20);
    double endYaw = base::Dubins3DStateSpace::degToRad(90);

    runTest(testName, desired, startX, startY, startZ, startPitch, startYaw, endX, endY, endZ, endPitch, endYaw);
}
