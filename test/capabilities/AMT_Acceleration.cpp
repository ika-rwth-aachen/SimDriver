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
// Created by Jens Klimke on 2020-02-23.
// Contributors:
//
// AMT_Acceleration.cpp
//


#include <simcore/value/SignalTube.h>
#include "AgentModelAdapter.h"


/**
 * Test collection #2: Acceleration
 * Here, the acceleration reaction is tested.
 *
 * The scenarios are given in Gherkin syntax. The placeholder values are defined by parameters:
 *
 */
struct AMT_Acceleration : public AgentModelAdapter {

    double targetVelocity = 27.8;
    double initialVelocity = 0.0;

    AMT_Acceleration() = default;
    ~AMT_Acceleration() override = default;


    void setupTest(std::string &&label, unsigned int no) {

        // setup end conditions
        setup.endTime = 30.0;
        setup.endDistance = INFINITY;

        // create simulation
//        setup.logFile = sim::fnc::string_format("AMT_AccelerationControl_%s.json", label.c_str());
        setup.logTitle = sim::fnc::string_format("Test 2.%d", no);

        // create simulation
        create();

        // set initial speed
        vehState->v = initialVelocity;

    }


};


/**
 * 1. Test: Acceleration Control - ReachSpeed
 *
 * Scenario:
 *  Given is a vehicle driver unit
 *  And the vehicle is standing still
 *  And the initial pedal value is <init_pedal>
 *  And the driver having a comfort target velocity of <target_speed>
 *  When simulation ran for <sim_time> seconds
 *  Then the vehicle shall have reached a velocity of <target_speed>
 *  And the acceleration shall be almost zero.
 *
 * Values:
 *  sim_time = 30 s
 *  init_pedal = _parameters.stopping.pedalDuringStanding
 *  target_speed = 27.8 (~100 kph) (+- 1 m/s)
 *  value = _parameters.stopping.pedalDuringStanding (+- 1e-3)
 */
TEST_F(AMT_Acceleration, ReachSpeed) {

    targetVelocity = 6.0;

    // set pre
    this->pre = [this] (double, double) {
        drParam->velocity.vComfort = targetVelocity;
    };

    // set post
    this->post = [this] (double, double) {
        EXPECT_DOUBLE_EQ(targetVelocity, drState->conscious.velocity.local);
    };

    // setup test
    setupTest(test_info_->name(), 1);

    // initial pedal value
    vehInput->pedal = drParam->stop.pedalDuringStanding;

    // run simulation
    run();

    // check pedal value
    ASSERT_NEAR(targetVelocity, this->vehState->v, 1.0);
    ASSERT_NEAR(0.0, this->vehState->a, 0.1);

}


/**
 * 2. Test: Acceleration Control - ReachSpeedVarious
 *
 * Scenario:
 *  Given is a vehicle driver unit
 *  And the vehicle is travelling with an initial velocity of <init_speed>
 *  And the driver having a comfort target velocity of <target_speed>
 *  When simulation ran for <sim_time> seconds
 *  Then the vehicle shall have reached a velocity of <target_speed>
 *  And the acceleration shall be almost zero.
 *
 * Values:
 *  sim_time = 30 s
 *  init_speed = {0.0, 1.0, 5.0, 10.0, 30.0} m/s
 *  target_speed = {0.0, 1.0, 5.0, 30.0} m/s (+- 1 m/s)
 *  value = _parameters.stopping.pedalDuringStanding (+- 1e-3)
 */
TEST_F(AMT_Acceleration, ReachSpeedVarious) {

    // set time step size to 0.1
    setup.stepSize = 0.1;

    // setup test
    setupTest(test_info_->name(), 2);

    std::vector<double> _v0 = {0.0, 1.0, 5.0, 10.0, 30.0};
    std::vector<double> _v1 = {0.0, 1.0, 5.0, 30.0, 0.0};

    // iterate over tests
    for(double v0 : _v0) {

        for(double v1 : _v1) {

            // set target velocity
            targetVelocity = v1;
            initialVelocity = v0;

            // set pre
            this->pre = [this] (double, double) {
                drParam->velocity.vComfort = targetVelocity;
            };

            // set pre
            this->post = [this] (double simTime, double) {

//                // after delta v seconds, the speed shall be reached
//                if(simTime > std::abs(initialVelocity - targetVelocity)) {
//                    EXPECT_NEAR(targetVelocity, vehState->v, targetVelocity * 0.1);
//                }

            };

            // reset simulation
            vehicle->reset();
            vehState->v = initialVelocity;

            // run simulation
            run();

            // check pedal value
            ASSERT_NEAR(targetVelocity, this->vehState->v, 0.1);
            ASSERT_NEAR(0.0, this->vehState->a, 0.1);

        }

    }

}



