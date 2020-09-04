// Copyright (c) 2019-2020 Jens Klimke <jens.klimke@rwth-aachen.de>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// Created by Jens Klimke on 2020-06-05.
//

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma ide diagnostic ignored "cert-err58-cpp"

#include <gtest/gtest.h>
#include <VehicleModel.h>
#include <Logging/UDPLogger.h>
#include <cmath>

#define EPS_VELOCITY 1e-3
#define EPS_ACCELERATION 1e-4

class VehicleModelTest : public ::testing::Test, public VehicleModel {

public:

    double _vMax  = 250.0 / 3.6;
    double _vIdle = 4.0;

    UDPLogger *logger = nullptr;

    VehicleModelTest() = default;
    ~VehicleModelTest() override = default;


    void SetUp() override {

        // set vehicle parameters
        parameters.maxWheelAngle = 30.0 * M_PI / 180.0;
        parameters.wheelBase = 3.0;
        parameters.maxDriveAcc = 10.0;
        parameters.maxBrakeAcc = 10.0;
        parameters.sideDrift = 0.0;

        // set long parameters
        this->setParametersByVelocities(_vMax, 2.0, _vIdle);

        // reset vehicle model
        reset();

    }


    void setParametersByVelocities(double vLimit, double vNomial, double vIdle) {

        // set max. relative drive power
        parameters.maxRelDrivePower = parameters.maxDriveAcc * vNomial;

        // set external acceleration parameters
        parameters.longExternalAcc[0] = 0.0;
        parameters.longExternalAcc[1] = 0.0;
        parameters.longExternalAcc[2] = parameters.maxRelDrivePower / (vLimit * vLimit * vLimit);

        // set idle pedal value
        parameters.idlePedal = parameters.longExternalAcc[2] * (vIdle * vIdle * vIdle) / parameters.maxRelDrivePower;

    }

    void run(double endTime = 1000.0) {

        // logging
        createLog();

        // init
        double timeStepSize = 0.01;
        double t = 0.0;
        unsigned long i = 0;

        // loop
        do {

            // step
            step(timeStepSize);

            // write log data
            if(i % 100 == 0)
                this->logger->write(t);

            // increase time step
            t += timeStepSize;

            ++i;

        } while(t < endTime);

        // delete logger
        delete this->logger;

    }


    void createLog() {

        // create reset flag
        auto name = std::string(::testing::UnitTest::GetInstance()->current_test_info()->name());
        std::string reset = R"({"reset":")" + name + R"(","title":")" + name + R"(","maxSamplePoints":1000,"keys":["v"]})";

        // create logger
        this->logger = new UDPLogger("localhost", "3001");
        this->logger->send(reset);
        this->logger->registerValue("owner", &reset);
        this->logger->registerValue("x", &this->state.position.x);
        this->logger->registerValue("y", &this->state.position.y);
        this->logger->registerValue("v", &this->state.v);

    }

};


TEST_F(VehicleModelTest, MaxSpeed) {

    // set pedal and steering value
    input.pedal = 1.0;
    input.steer = 0.0;

    // run simulation
    run();

    // check results
    EXPECT_NEAR(  0.0, state.a, EPS_ACCELERATION);
    EXPECT_NEAR(_vMax, state.v, EPS_VELOCITY);

}


TEST_F(VehicleModelTest, IdleSpeed) {

    // set pedal and steering value
    input.pedal = 0.0;
    input.steer = 0.0;

    // run simulation
    run();

    // check results
    EXPECT_NEAR(   0.0, state.a, EPS_ACCELERATION);
    EXPECT_NEAR(_vIdle, state.v, EPS_VELOCITY);

}


TEST_F(VehicleModelTest, IdleSpeedWithStdParams) {

    // set pedal and steering value
    input.pedal = 0.0;
    input.steer = 0.0;

    // run simulation
    run();

    EXPECT_NEAR(    0.0, state.a,  1e-3);
    EXPECT_NEAR(    7.5, state.v,  1.0);
    EXPECT_NEAR(    0.1, state.ds, 1.0);
    EXPECT_NEAR( 7085.0, state.s,  1.0);

}


TEST_F(VehicleModelTest, SteadyTurn) {

    // set pedal and steering value
    input.pedal = 0.1;
    input.steer = 0.1;

    // run simulation
    run();

    EXPECT_NEAR(0.0,     state.a, 1e-4);
    EXPECT_NEAR(19.5095, state.v, 1e-4);

    EXPECT_NEAR(-60.0, state.position.x, 1.0);
    EXPECT_NEAR( 66.0, state.position.y, 1.0);
    EXPECT_NEAR(  0.5, state.dPsi, 1.0);
    EXPECT_NEAR(319.0, state.psi,  1.0);

}


TEST_F(VehicleModelTest, Reset) {

    // set pedal and steering value
    input.pedal = 0.1;
    input.steer = 0.1;


    // run simulation
    run();

    EXPECT_LT(0.0, state.v);
    EXPECT_LT(0.0, state.s);
    EXPECT_GT(0.0, state.position.x);
    EXPECT_LT(0.0, state.position.y);
    EXPECT_LT(0.0, state.psi);


    // reset
    reset();

    EXPECT_DOUBLE_EQ(0.0, state.v);
    EXPECT_DOUBLE_EQ(0.0, state.s);
    EXPECT_DOUBLE_EQ(0.0, state.position.x);
    EXPECT_DOUBLE_EQ(0.0, state.position.y);
    EXPECT_DOUBLE_EQ(0.0, state.psi);


}



TEST(VehicleModelDataTest, Access) {

    VehicleModel vehicle{};
    const VehicleModel *vehPtr = &vehicle;

    auto i = vehicle.getInput();
    auto s = vehicle.getState();
    auto p = vehicle.getParameters();

    auto ic = vehPtr->getInput();
    auto sc = vehPtr->getState();
    auto pc = vehPtr->getParameters();

    EXPECT_EQ(i, ic);
    EXPECT_EQ(s, sc);
    EXPECT_EQ(p, pc);

}

#pragma clang diagnostic pop