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
#include <VehicleModel/VehicleModel.h>
#include <cmath>


class VehicleModelTest : public ::testing::Test, public VehicleModel {

public:

    VehicleModelTest() = default;
    ~VehicleModelTest() override = default;


    void SetUp() override {

        // set vehicle parameters
        parameters.steerTransmission  = 0.5;
        parameters.wheelBase          = 3.0;
        parameters.cwA                = {0.6, 1.2};
        parameters.mass               = 1.5e3;
        parameters.powerMax           = 1.0e5;
        parameters.forceMax           = 1.5e4;
        parameters.idle               = 0.05;
        parameters.rollCoefficient[0] = 4.0 * 9.91e-3;
        parameters.rollCoefficient[1] = 4.0 * 1.95e-5;
        parameters.rollCoefficient[2] = 4.0 * 1.76e-9;
        parameters.size.x             = 5.0;
        parameters.size.y             = 2.2;
        parameters.driverPosition.x   = 0.5;
        parameters.driverPosition.y   = 0.5;

    }

    void run(double endTime = 1000.0) {

        // init
        double timeStepSize = 0.01;
        double t = 0.0;

        // loop
        do {

            // step
            step(timeStepSize);

            // increase time step
            t += timeStepSize;

        } while(t < endTime);

    }

};


TEST_F(VehicleModelTest, MaxSpeed) {

    // set pedal and steering value
    input.pedal = 1.0;
    input.steer = 0.0;

    // set parameters
    parameters.rollCoefficient[0] = 0.0;
    parameters.rollCoefficient[1] = 0.0;
    parameters.rollCoefficient[2] = 0.0;

    // calculates KPIs of the vehicle model
    auto cwa = 0.5 * 1.2041 * parameters.cwA.x;
    auto rhs = std::pow(parameters.powerMax / cwa, 1.0 / 3.0);

    // run simulation
    run();

    EXPECT_NEAR(0.0, state.a, 1e-4);
    EXPECT_NEAR(rhs, state.v, 1e-4);

}


TEST_F(VehicleModelTest, PushingWind) {

    // set pedal and steering value
    input.pedal = 0.0;
    input.steer = 0.0;
    input.slope = {0.0, 0.0};
    input.windSpeed = {10.0, 0.0};

    // run simulation
    run();

    EXPECT_NEAR(    0.0, state.a,  1e-3);
    EXPECT_NEAR(    8.0, state.v,  1.0);
    EXPECT_NEAR(    0.0, state.ds, 1.0);
    EXPECT_NEAR( 7448.0, state.s,  1.0);

}


TEST_F(VehicleModelTest, MaxSpeedWithStdParams) {

    // set pedal and steering value
    input.pedal = 1.0;
    input.steer = 0.0;
    input.slope = {0.1, 0.0};
    input.windSpeed = {12.0, 0.0};

    // run simulation
    run();

    EXPECT_NEAR(    0.0, state.a,  1e-3);
    EXPECT_NEAR(   41.5, state.v,  1.0);
    EXPECT_NEAR(    0.0, state.ds, 1.0);
    EXPECT_NEAR(41016.0, state.s,  1.0);

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