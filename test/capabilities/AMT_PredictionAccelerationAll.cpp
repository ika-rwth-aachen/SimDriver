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


#include <simcore/utils/functions.h>
#include <simcore/value/SignalTube.h>
#include "AgentModelAdapter.h"


/**
 * Test collection #3: Acceleration due to predictive behavior
 * Here, the acceleration reaction is tested.
 *
 * The scenarios are given in Gherkin syntax. The placeholder values are defined by parameters:
 *
 */
struct AMT_PredictionAccelerationAll : public AgentModelAdapter, public SignalTube {

    double initialVelocity = 20.0;
    double targetVelocity  =  5.0;
    double followVelocity  = 10.0;

    double followDistance = 300.0;
    double stepPosition   = 750.0;
    double stopPosition   = 800.0;

    double ds = 0.0;

    double *v = nullptr;
    double *s = nullptr;

    AMT_PredictionAccelerationAll() = default;
    ~AMT_PredictionAccelerationAll() override = default;


    void setupTest(std::string &&label, unsigned int no) {

        // setup end conditions
        setup.endTime = 120.0;
        setup.endDistance = 1200.0;

        // create simulation
//        setup.logFile = sim::fnc::string_format("AMT_PredictionAccelerationControl_%s.json", label.c_str());
        setup.logTitle = sim::fnc::string_format("Test 3.%d", no);

        // create simulation
        create();

        // set initial pedal value, to avoid controller to settle too long
        vehInput->pedal = 0.1;
        vehState->v = initialVelocity;

        // set v and s
        v = &vehState->v;
        s = &vehState->s;

        // add distance to log
        jsonLogger.addValue("ds", &ds);

    }


    void preStep(double simTime, double deltaTime) override {

        // save distance
        followDistance += followVelocity * deltaTime;

        // run pre step of GenericTest
        AgentModelAdapter::preStep(simTime, deltaTime);

    }


};


/**
 * 6. Test: Acceleration Control - All together
 *
 * Scenario:
 *  Given is a vehicle driver unit
 *  And the vehicle is travelling with an initial velocity of <init_velocity>
 *  And the driver having a comfort target velocity of <init_velocity>
 *  And the target speed at the position <speed_step_position> shall be <target_velocity>
 *  And the object to be followed is at the initial position <init_follow_position>
 *  And the object to be followed has a constant velocity of <follow_velocity>
 *  And the target stop is located at the position <target_stop_position>
 *  When simulation ran for <sim_time> seconds
 *
 *
 * Values:
 *  sim_time = 60 s
 *  init_velocity = 20.0 m/s
 *  speed_step_position = 500.0 m
 *  target_velocity = 5.0 m/s
 *  init_follow_position = 300.0 m
 *  follow_velocity = 10.0 m/s
 *  target_stop_position = 600.0 m
 *
 */
TEST_F(AMT_PredictionAccelerationAll, AllTogether) {

    // set pre
    this->pre = [this] (double, double) {

        // TODO: Test for everything together

    };

    // setup test
    setupTest(test_info_->name(), 6);

    // run simulation
    run();

}
