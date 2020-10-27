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
// Created by Jens Klimke on 2020-04-05.
// Contributors:
//
// AMT_StopSigns.cpp
//

#include "AgentModelAdapter.h"

/**
 * Test collection #10: Stop signs
 * Here, the reaction on stop signs is tested.
 */
struct AMT_StopSigns : public AgentModelAdapter {

    double initialVelocity = 15.0;
    double ds = 0.0;

    AMT_StopSigns() = default;
    ~AMT_StopSigns() override = default;

    void setupTest(std::string &&label, unsigned int no) {

        // setup end conditions
        setup.endTime = 60.0;
        setup.endDistance = INFINITY;

        // create simulation
        setup.logTitle = sim::fnc::string_format("Test 10.%d", no);

        // create simulation
        create();

        // set initial speed
        vehState->v = initialVelocity;
        drParam->velocity.vComfort = initialVelocity;

        if(EN_LOGGING) {

            // add distance to log
            jsonLogger.addValue("ds", &ds);

            // add to figure
            plotLogger.addFigure("distance", "Distance", "time t [s]", "distance ds [m]", "time", "ds");

            // add to figure
            plotLogger.addFigure("velocity_over_s", "Velocity over position", "long. position [m]",
                                 "velocity [m/s]", "s", "v");

        }

    }



};


/**
 * 1. Test: Stop sign reaction - Stop ahead
 *
 * Scenario:
 *  Given is a vehicle driver unit
 *   And the vehicle is travelling with an initial velocity of <init_speed>
 *   And a stop sign is located at the position <pos_stop>
 *  When the vehicle has stopped
 *   Then the vehicle shall have travelled a distance of <dist_stop>.
 *
 * Values:
 *  sim_time = 30 s
 *  init_speed = 15 m/s
 *  pos_stop = 400 m
 *  dist_stop = 395.5 m (+- 0.5)
 *
 */
TEST_F(AMT_StopSigns, StopAhead) {

    // setup test
    setupTest(test_info_->name(), 1);

    this->pre = [this](double simTime, double timeStep) {

        // calculate distance
        ds = 399.5 - vehState->s;

        // set input
        drInput->signals[0].ds = ds;
        drInput->signals[0].type = agent_model::SignalType::SIGNAL_STOP;
        drInput->signals[0].id = 1;

    };

    this->post = [this](double simTime, double timeStep) {

        if(vehState->v == 0.0)
            EXPECT_NEAR(395.5, vehState->s, 0.5);

    };

    // run test
    run();

}


/**
 * 2. Test: Stop sign reaction - Stop ahead
 *
 * Scenario:
 *  Given is a vehicle driver unit
 *   And the vehicle is travelling with an initial velocity of <init_speed>
 *   And a stop sign is located at the position <pos_stop1>
 *   And a stop sign is located at the position <pos_stop2>
 *  When the vehicle has travelled less than 500 m and stopped
 *   Then the vehicle shall have travelled a distance of <dist_stop1>
 *  When the vehicle has travelled more than 500 m and stopped
 *   Then the vehicle shall have travelled a distance of <dist_stop2>.
 *
 * Values:
 *  sim_time = 30 s
 *  init_speed = 15 m/s
 *  pos_stop1 = 400 m
 *  pos_stop2 = 600 m
 *  dist_stop = 395.5 m (+- 0.5)
 *  dist_stop = 595.5 m (+- 0.5)
 *
 */
TEST_F(AMT_StopSigns, TwoStopsAhead) {

    // setup test
    setupTest(test_info_->name(), 1);

    this->pre = [this](double simTime, double timeStep) {

        // calculate distance
        ds = 399.5 - vehState->s;

        // set input
        drInput->signals[0].ds = ds + 200.0;
        drInput->signals[0].type = agent_model::SignalType::SIGNAL_STOP;
        drInput->signals[0].id = 2;

        // set input
        drInput->signals[1].ds = ds;
        drInput->signals[1].type = agent_model::SignalType::SIGNAL_STOP;
        drInput->signals[1].id = 1;

    };

    this->post = [this](double simTime, double timeStep) {

        if(vehState->v == 0.0 && vehState->s > 500.0)
            EXPECT_NEAR(595.5, vehState->s, 0.5);
        else if(vehState->v == 0.0)
            EXPECT_NEAR(395.5, vehState->s, 0.5);

    };

    // run test
    run();

}



