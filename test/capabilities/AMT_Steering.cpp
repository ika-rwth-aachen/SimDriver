// Copyright (c) 2019 Institute for Automotive Engineering (ika), RWTH Aachen University. All rights reserved.
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
// Created by Jens Klimke on 2020-03-11.
// Contributors:
//
// AMT_Steering.cpp
//

#include <simcore/functions.h>
#include "AgentModelAdapter.h"

/**
 * Test collection #4: Steering control
 * Here, the steering reaction is tested.
 */
struct AMT_Steering : public AgentModelAdapter {

    AMT_Steering() = default;
    ~AMT_Steering() override = default;

    double initialVelocity = 10.0;
    double initialSteeringAngle = 0.0;

    void initialize(double initTime) override {

        AgentModelAdapter::initialize(initTime);
        vehState->v = initialVelocity;
        vehState->delta = initialSteeringAngle;

    }

    void setupTest(std::string &&label, unsigned int no) {

        // setup end conditions
        setup.endTime = 100.0;
        setup.endDistance = INFINITY;

        // create simulation
//        setup.logFile = sim::fnc::string_format("AMT_SteeringControl_%s.json", label.c_str());;
        setup.logTitle = sim::fnc::string_format("Test 4.%d", no);;

        // create simulation
        create();

    }

    void preStep(double simTime, double deltaTime) override {

        // set desired velocity
        drParam->velocity.vComfort = initialVelocity;

        // execute pre
        AgentModelAdapter::preStep(simTime, deltaTime);

    }


};


/**
 * 1. Test: Steering Control - Various curvatures
 *
 * Scenario:
 *  Given is a vehicle driver unit
 *  And the vehicle is moving with v = <init_speed>
 *  And the driver having a target velocity of <init_speed>
 *  And the steering is straight
 *  And the driver having a target curvature of <target_curvature>
 *  When simulation ran for <sim_time> seconds
 *  Then the vehicle shall move with v = <init_speed>
 *  And the curvature of the vehicle shall be <target_curvature>.
 *
 * Values:
 *  sim_time = 10 s
 *  init_speed = 10 m/s
 *  target_curvature = (-0.05 .. 0.05) 1/m in 21 steps
 */
TEST_F(AMT_Steering, ControlCurvature) {

    // setup test
    setupTest(test_info_->name(), 1);

    for(int i = 10; i >= -10; --i) {

        double kappa = 0.005 * i;

        // set pre
        this->pre = [this, kappa](double, double) {
            subconscious->kappa = kappa;
        };

        // run simulation
        run();

        // check pedal value
        EXPECT_NEAR(10.0, this->vehState->v, 1e-3);
        EXPECT_NEAR(kappa, this->vehState->kappa, 1e-3);

    }

}


/**
 * 2. Test: Steering Control - Increasing speed
 *
 * Scenario:
 *  Given is a vehicle driver unit
 *  And the vehicle is initially standing still
 *  And the steering is straight
 *  And the driver having a target acceleration of <target_acc>
 *  And the driver having a target curvature of <target_curvature>
 *  When simulation ran for <sim_time> seconds
 *  Then the vehicle shall move with v = <final_speed>
 *  And the curvature of the vehicle shall be <target_curvature>.
 *
 * Values:
 *  sim_time = 10 s
 *  target_acc = 0.1 m/s^2
 *  final_speed = 10 +- 1 m/s
 *  target_curvature = 0.01 +- 1e5
 */
TEST_F(AMT_Steering, ControlCurvatureDuringAccleration) {

    // set init speed to zero
    initialVelocity = 0.0;

    // setup test
    setupTest(test_info_->name(), 2);

    // set pre
    this->pre = [this](double, double) {
        subconscious->kappa = 0.01;
        subconscious->a = 0.1;
    };

    // set post
    this->post = [this](double simTime, double) {

        if(simTime > 10.0) {

            // check after a short while if curvature is controlled
            EXPECT_NEAR(0.01, vehState->kappa, 1e-3);

        } else {

            // between zero and 0.01
            EXPECT_GE(vehState->kappa, 0.0);
            EXPECT_LE(vehState->kappa, 0.011);

        }

    };

    // run simulation
    run();

    // check pedal value
    EXPECT_NEAR(10.0,  this->vehState->v,     1.0);
    EXPECT_NEAR( 0.01, this->vehState->kappa, 1e-5);

}