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
// Created by Jens Klimke on 2020-02-17.
// Contributors:
//
// AMT_Pedal.cpp
//


#include <simcore/utils/functions.h>
#include "AgentModelAdapter.h"


/**
 * Test collection #1: Pedal control
 * Here, the pedal reaction is tested.
 */
struct AMT_Pedal : public AgentModelAdapter {

    double initialVelocity =  0.0;
    double targetVelocity  = 10.0;
    double initialPedal    =  0.0;
    double stopTime        = 10.0;

    AMT_Pedal() = default;
    ~AMT_Pedal() override = default;


    void reset() {

        // reset state
        vehicle->reset();

        // set pedal and speed
        vehState->v = initialVelocity;
        vehInput->pedal = initialPedal;

    }

    void setupTest(std::string &&label, unsigned int no) {

        // setup end conditions
        setup.endTime = stopTime;
        setup.endDistance = INFINITY;
        setup.stepSize = 0.01;

        // create simulation
//        setup.logFile = sim::fnc::string_format("AMT_PedalControl_%s.json", label.c_str());;
        setup.logTitle = sim::fnc::string_format("Test 1.%d", no);;

        // create simulation
        create();
        reset();

    }

    void preStep(double simTime, double deltaTime) override {

        // set local velocity and desired velocity
        drParam->velocity.vComfort = targetVelocity;

        // execute pre
        AgentModelAdapter::preStep(simTime, deltaTime);

        // check the initial values in the first step
        if(simTime <= 1e-12) {
            EXPECT_DOUBLE_EQ(initialPedal,    vehInput->pedal);
            EXPECT_DOUBLE_EQ(initialVelocity, vehState->v);
        }

    }


};


/**
 * 1. Test: Pedal Control - Hold Vehicle in standstill
 *
 * Scenario:
 *  Given is a vehicle driver unit
 *  And the vehicle is standing still
 *  And the initial pedal value is <init_pedal>
 *  And the driver having a target velocity of <target_speed>
 *  And the stopping flag is set
 *  When simulation ran for <sim_time> seconds
 *  Then the driver shall have controlled the pedal to a value of <value>
 *  And the vehicle shall have stopped
 *  And the acceleration is zero.
 *
 * Values:
 *  sim_time = 10 s
 *  target_speed = 10 m/s
 *  init_pedal = _parameters.stopping.pedalDuringStanding * 0.33
 *  value = _parameters.stopping.pedalDuringStanding (+- 1e-3)
 */
TEST_F(AMT_Pedal, HoldVehicle) {

    // set initial pedal
    initialPedal = -_param.stop.pedalDuringStanding * 0.33;

    // set pre
    this->pre = [this] (double, double) {
        conscious->stop.standing = true;
    };

    // setup test
    setupTest(test_info_->name(), 1);

    // run simulation
    run();

    // check pedal value
    ASSERT_NEAR(-0.3, this->vehInput->pedal, 1e-3);
    ASSERT_NEAR(0.0, this->vehState->v, 1e-3);
    ASSERT_NEAR(0.0, this->vehState->a, 1e-3);

}


/**
 * 2. Test: Pedal Control - StopVehicleByPedal
 *
 * Scenario:
 *  Given is a vehicle driver unit
 *  And the vehicle is moving with v = <init_speed>
 *  And the initial pedal value is neutral
 *  And the driver having a target velocity of <target_speed>
 *  And the stopping flag is set continuously
 *  When simulation ran for <sim_time> seconds
 *  Then the driver shall have controlled the pedal to a value of <value>
 *  And the vehicle shall have stopped
 *  And the acceleration is zero.
 *
 * Values:
 *  sim_time = 10 s
 *  init_speed = 10 m/s
 *  target_speed = 10 m/s
 *  value = _parameters.stopping.pedalDuringStanding (+- 1e-3)
 */
TEST_F(AMT_Pedal, StopVehicleByPedal) {

    // set initial to target
    initialVelocity = 10.0;

    // set pre
    this->pre = [this] (double, double) {
        conscious->stop.standing = true;
    };

    // setup test
    setupTest(test_info_->name(), 2);

    // run simulation
    run();

    // check pedal value
    ASSERT_NEAR(-0.3, this->vehInput->pedal, 1e-3);
    ASSERT_NEAR(0.0, this->vehState->v, 1e-3);
    ASSERT_NEAR(0.0, this->vehState->a, 1e-3);

}


/**
 * Test 3 a) and b): Pedal Control - ReachAcceleration
 *
 * Scenario:
 *  Given is a vehicle driver unit
 *  And the vehicle is moving with v = <init_speed>
 *  And the initial pedal is set to <init_pedal>
 *  And the driver having a target velocity of <target_speed>
 *  And the desired acceleration is set to <target_acc> continuously
 *  When simulation ran for <sim_time> seconds
 *  Then the acceleration shall have reached <target_acc>
 *
 * Values:
 *  sim_time = 5 s
 *  init_speed = 2 m/s | 30 m/s
 *  init_pedal = 1 | 1
 *  target_speed = 10 m/s | 30 m/s
 *  target_acc = 1.5 m/s^2 (+- 1e-3) | -4.0 m/s^2 (+- 1e-3)
 *
 *  Test 3.a)
 */
TEST_F(AMT_Pedal, ReachAcceleration_a) {

    // set stop time and initial speed
    stopTime = 5.0;
    initialVelocity = 10.0;
    initialPedal = 1.0;

    // setup test
    setupTest(test_info_->name(), 3);

    // injection
    this->pre = [this](double, double) {
        subconscious->a = 0.0;
    };

    // run simulation
    run();

    // check pedal value
    ASSERT_NEAR(0.0, this->vehState->a, 1e-2);

}


/**
 * Test 3.b)
 */
TEST_F(AMT_Pedal, ReachAcceleration_b) {

    // set stop time and initial speed
    stopTime = 5.0;
    initialVelocity = 30.0;
    initialPedal = 1.0;

    // setup test
    setupTest(test_info_->name(), 3);

    // injection
    this->pre = [this] (double, double) {
        subconscious->a = -4.0;
    };

    // reset
    reset();

    // run simulation
    run();

    // check pedal value
    ASSERT_NEAR(-4.0, this->vehState->a, 1e-2);

}


/**
 * Test 4: Pedal Control - DirectPedalControl
 *
 * Scenario:
 *  Given is a vehicle driver unit
 *  And the vehicle is standing still
 *  And the initial pedal value <init_pedal>
 *  And the driver having a target pedal value is set to <target_pedal>
 *  When simulation ran for <sim_time> seconds
 *  Then the pedal shall have reached the <target_pedal>
 *
 * Values:
 *  sim_time = 5 s | 50 s,
 *  init_pedal = 1.0
 *  target_pedal = -1.0 (99 %) | -1.0 (99.99 %);
 */
TEST_F(AMT_Pedal, DirectPedalControl) {

    // setup test
    initialPedal = 1.0;
    setupTest(test_info_->name(), 4);

    // set pre
    this->pre = [this](double, double) {
        subconscious->pedal = -1.0;
    };


    /** run simulation a) **/

    // reset and run simulation
    reset();
    run();

    // check pedal value
    ASSERT_NEAR(-1.0, vehInput->pedal, 1e-2);


    /** run simulation b) **/

    // reset and run simulation
    reset();
    run();

    // check pedal value
    ASSERT_NEAR(-1.0, vehInput->pedal, 1e-4);


}
