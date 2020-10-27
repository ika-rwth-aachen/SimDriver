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
// AMT_LateralControl.cpp
//

#include <simcore/functions.h>
#include "AgentModelAdapter.h"

/**
 * Test collection #6: Lateral control
 * Here, the steering reaction is tested.
 */
struct AMT_LateralControl : public AgentModelAdapter {

    AMT_LateralControl() = default;
    ~AMT_LateralControl() override = default;

    double targetVelocity = 10.0;
    double endTime = 30.0;

    struct {
        double x = 0.0;
        double y = 0.0;
    } pos{};


    void initialize(double initTime) override {

        AgentModelAdapter::initialize(initTime);

    }


    void setupTest(std::string &&label, unsigned int no) {

        // setup end conditions
        setup.endTime = endTime;
        setup.endDistance = INFINITY;
        setup.stepSize = 0.1;

        // create simulation
//        setup.logFile = sim::fnc::string_format("AMT_SteeringControl_%s.json", label.c_str());;
        setup.logTitle = sim::fnc::string_format("Test 6.%d", no);

        // create simulation
        create();


        if(EN_LOGGING) {

            // add logger
            jsonLogger.addValue("x0", &pos.x);
            jsonLogger.addValue("y0", &pos.y);
            plotLogger.trace("position", "ref", "x0", "y0", "auto", 2);

        }

        // set vehicle speed
        vehState->v = targetVelocity;
        vehState->psi = 0.0;
        vehInput->steer = 0.0;

    }


    void preStep(double simTime, double deltaTime) override {

        // execute pre
        AgentModelAdapter::preStep(simTime, deltaTime);

    }


};


/**
 * 1. Test: Lateral Control - Straight
 *
 * Scenario:
 *  Given is a vehicle driver unit
 *  And the vehicle is moving with a initial velocity of <init_speed>
 *  And the vehicle's yaw angle is exactly zero
 *  And the driver having a comfort target velocity of <init_speed>
 *  And all reference points are unset
 *  After each simulation step
 *  Then the vehicle shall move exactly in x-direction
 *
 * Values:
 *  sim_time = 30 s
 *  init_speed = 10.0 (+- 1 m/s)
 */
TEST_F(AMT_LateralControl, Straight) {

    // setup simulation
    setupTest(test_info_->name(), 1);

    // set pre
    this->pre = [this] (double simTime, double deltaTime) {

        // inject
        conscious->lateral.paths[0].factor = 1.0;
        conscious->lateral.paths[1].factor = 0.0;
        conscious->lateral.paths[2].factor = 0.0;

        conscious->lateral.paths[0].refPoints[0].x = INFINITY;
        conscious->lateral.paths[0].refPoints[0].y = 0.0;
        conscious->lateral.paths[0].refPoints[0].dx = 0.0;
        conscious->lateral.paths[0].refPoints[0].dy = 0.0;

        conscious->lateral.paths[0].refPoints[1].x = INFINITY;
        conscious->lateral.paths[0].refPoints[1].y = 0.0;
        conscious->lateral.paths[0].refPoints[1].dx = 0.0;
        conscious->lateral.paths[0].refPoints[1].dy = 0.0;

        // set target speed
        drParam->velocity.vComfort = targetVelocity;

    };

    this->post = [this] (double simTime, double deltaTime) {

        EXPECT_DOUBLE_EQ(0.0, vehState->position.y);
        EXPECT_DOUBLE_EQ(0.0, vehState->psi);

        EXPECT_NEAR(10.0, vehState->v, 1.0);

        EXPECT_EQ(INFINITY, drState->conscious.lateral.paths[0].refPoints[0].x);
        EXPECT_DOUBLE_EQ(0.0, drState->conscious.lateral.paths[0].refPoints[0].y);

        EXPECT_EQ(INFINITY, drState->conscious.lateral.paths[0].refPoints[1].x);
        EXPECT_DOUBLE_EQ(0.0, drState->conscious.lateral.paths[0].refPoints[1].y);

    };

    // run simulation
    run();

}


/**
 * 2. Test: Lateral Control - Circle
 *
 * Scenario:
 *  Given is a vehicle driver unit
 *  And the vehicle is moving with a initial velocity of <init_speed>
 *  And the driver having a comfort target velocity of <init_speed>
 *  And the first reference point is constantly set to <xy_ref>
 *  And all reference points except the first one are unset
 *  After 10 seconds of simulation and after each simulation step
 *  Then the vehicle shall move with a radius of <radius>
 *
 * Values:
 *  sim_time = 30 s
 *  init_speed = 10.0 m/s
 *  xy_ref = (1.0 m, 0.1 m)
 *  radius = 62.0 m (+- 4 m)
 */
TEST_F(AMT_LateralControl, Circle) {

    // setup simulation
    setupTest(test_info_->name(), 2);

    // set pre
    this->pre = [this] (double simTime, double deltaTime) {

        // inject
        conscious->lateral.paths[0].factor = 1.0;
        conscious->lateral.paths[1].factor = 0.0;
        conscious->lateral.paths[2].factor = 0.0;

        conscious->lateral.paths[0].refPoints[0].x = 1.0;
        conscious->lateral.paths[0].refPoints[0].y = -0.1;
        conscious->lateral.paths[0].refPoints[0].dx = 0.0;
        conscious->lateral.paths[0].refPoints[0].dy = 0.0;

        conscious->lateral.paths[0].refPoints[1].x = INFINITY;
        conscious->lateral.paths[0].refPoints[1].y = 0.0;
        conscious->lateral.paths[0].refPoints[1].dx = 0.0;
        conscious->lateral.paths[0].refPoints[1].dy = 0.0;

        // set target speed
        drParam->velocity.vComfort = targetVelocity;

    };

    this->post = [this] (double simTime, double deltaTime) {

        if(simTime > 10.0)
            EXPECT_NEAR(62.0, -1.0 / vehState->kappa, 4.0);

    };

    // run simulation
    run();

}


/**
 * 3. Test: Lateral Control - Sinus Track
 *
 * Scenario:
 *  Given is a vehicle driver unit
 *  And the vehicle is moving with a initial velocity of <init_speed>
 *  And the driver having a comfort target velocity of <init_speed>
 *  And the first reference point is constantly set to a sinus track with the parameters <sin_param>
 *  And all reference points except the first one are unset
 *  After 2 seconds of simulation and after each simulation step
 *  Then the vehicle have a deviation from the sinus track in y-direction of max. <dist_max>
 *
 * Values:
 *  sim_time = 30 s
 *  init_speed = 10.0 m/s
 *  sin_param = (amplitude=1.0 m, wave_length=1/100 m)
 *  dist_max = 0.1 m
 */
TEST_F(AMT_LateralControl, SinusTrack) {

    // setup simulation
    setupTest(test_info_->name(), 3);

    // scale for the x position
    double d = 2.0 * M_PI / 100.0;

    // set pre
    this->pre = [this, d] (double simTime, double deltaTime) {

        // get position (one meter ahead)
        pos.x = vehState->position.x + 1.0 * targetVelocity;
        pos.y = 1.0 * sin(d * pos.x);

        // subtract ego position
        double x = pos.x - vehState->position.x;
        double y = pos.y - vehState->position.y;

        // rotation
        double c = cos(-vehState->psi);
        double s = sin(-vehState->psi);

        // calculate point
        double x0 = c * x - s * y;
        double y0 = s * x + c * y;

        // inject
        conscious->lateral.paths[0].factor = 1.0;
        conscious->lateral.paths[1].factor = 0.0;
        conscious->lateral.paths[2].factor = 0.0;

        conscious->lateral.paths[0].refPoints[0].x  = x0;
        conscious->lateral.paths[0].refPoints[0].y  = y0;
        conscious->lateral.paths[0].refPoints[0].dx = 0.0;
        conscious->lateral.paths[0].refPoints[0].dy = 0.0;

        conscious->lateral.paths[0].refPoints[1].x  = INFINITY;
        conscious->lateral.paths[0].refPoints[1].y  = 0.0;
        conscious->lateral.paths[0].refPoints[1].dx = 0.0;
        conscious->lateral.paths[0].refPoints[1].dy = 0.0;

        conscious->lateral.paths[1].refPoints[0].x = INFINITY;
        conscious->lateral.paths[1].refPoints[0].y = 0.0;
        conscious->lateral.paths[1].refPoints[0].dx = 0.0;
        conscious->lateral.paths[1].refPoints[0].dy = 0.0;

        conscious->lateral.paths[1].refPoints[1].x = INFINITY;
        conscious->lateral.paths[1].refPoints[1].y = 0.0;
        conscious->lateral.paths[1].refPoints[1].dx = 0.0;
        conscious->lateral.paths[1].refPoints[1].dy = 0.0;

        conscious->lateral.paths[2].refPoints[0].x = INFINITY;
        conscious->lateral.paths[2].refPoints[0].y = 0.0;
        conscious->lateral.paths[2].refPoints[0].dx = 0.0;
        conscious->lateral.paths[2].refPoints[0].dy = 0.0;

        conscious->lateral.paths[2].refPoints[1].x = INFINITY;
        conscious->lateral.paths[2].refPoints[1].y = 0.0;
        conscious->lateral.paths[2].refPoints[1].dx = 0.0;
        conscious->lateral.paths[2].refPoints[1].dy = 0.0;

        // set target speed
        drParam->velocity.vComfort = targetVelocity;

    };


    this->post = [this, d] (double simTime, double deltaTime) {

        if(simTime > 2.0) {

            // get position (at the current position)
            double y = 1.0 * sin(d * vehState->position.x);
            EXPECT_NEAR(y, vehState->position.y, 0.1);

        }

    };

    // run simulation
    run();

}


/**
 * 4. Test: Lateral Control - Compensate offset
 *
 * Scenario:
 *  Given is a vehicle driver unit
 *  And the vehicle is moving with a initial velocity of <init_speed>
 *  And the driver having a comfort target velocity of <init_speed>
 *  And the first reference point is constantly set to the x-axis one second ahead of the vehicle
 *  And the second reference point is constantly set to the x-axis two second ahead of the vehicle
 *  When simulation ran for <sim_time> seconds
 *  Then the vehicle shall have a deviation from the x-axis of maximum <dist_max>
 *
 * Values:
 *  sim_time = 30 s
 *  init_speed = 10.0 m/s
 *  sim_time = 5.0 s
 *  dist_max = 0.05 m
 */
TEST_F(AMT_LateralControl, CompensateOffset) {

    // set end time
    endTime = 5.0;

    // setup simulation
    setupTest(test_info_->name(), 4);

    // set initial position
    vehState->position.x = 0.0;
    vehState->position.y = 2.0;
    vehState->psi = M_PI_4;

    // set pre
    this->pre = [this] (double simTime, double deltaTime) {

        // get position (one meter ahead)
        pos.x = vehState->position.x + 1.0 * targetVelocity;
        pos.y = 0.0;

        // rotation
        double c = cos(-vehState->psi);
        double s = sin(-vehState->psi);

        // subtract ego position
        double x0 = pos.x - vehState->position.x;
        double y0 = pos.y - vehState->position.y;

        double x1 = 2.0 * targetVelocity;
        double y1 = -vehState->position.y;

        double dx = vehState->v * sin(vehState->psi);
        double dy = vehState->v * cos(vehState->psi);


        // inject
        conscious->lateral.paths[0].factor = 1.0;
        conscious->lateral.paths[1].factor = 0.0;
        conscious->lateral.paths[2].factor = 0.0;

        conscious->lateral.paths[0].refPoints[0].x  = c * x0 - s * y0;
        conscious->lateral.paths[0].refPoints[0].y  = s * x0 + c * y0;
        conscious->lateral.paths[0].refPoints[0].dx = dx;
        conscious->lateral.paths[0].refPoints[0].dy = dy;

        conscious->lateral.paths[0].refPoints[1].x  = c * x1 - s * y1;
        conscious->lateral.paths[0].refPoints[1].y  = s * x1 + c * y1;
        conscious->lateral.paths[0].refPoints[1].dx = dx;
        conscious->lateral.paths[0].refPoints[1].dy = dy;

        conscious->lateral.paths[1].refPoints[0].x = INFINITY;
        conscious->lateral.paths[1].refPoints[0].y = 0.0;
        conscious->lateral.paths[1].refPoints[0].dx = 0.0;
        conscious->lateral.paths[1].refPoints[0].dy = 0.0;

        conscious->lateral.paths[1].refPoints[1].x = INFINITY;
        conscious->lateral.paths[1].refPoints[1].y = 0.0;
        conscious->lateral.paths[1].refPoints[1].dx = 0.0;
        conscious->lateral.paths[1].refPoints[1].dy = 0.0;

        conscious->lateral.paths[2].refPoints[0].x = INFINITY;
        conscious->lateral.paths[2].refPoints[0].y = 0.0;
        conscious->lateral.paths[2].refPoints[0].dx = 0.0;
        conscious->lateral.paths[2].refPoints[0].dy = 0.0;

        conscious->lateral.paths[2].refPoints[1].x = INFINITY;
        conscious->lateral.paths[2].refPoints[1].y = 0.0;
        conscious->lateral.paths[2].refPoints[1].dx = 0.0;
        conscious->lateral.paths[2].refPoints[1].dy = 0.0;

        // set target speed
        drParam->velocity.vComfort = targetVelocity;

    };

    // run simulation
    run();

    // check
    EXPECT_NEAR(0.0, vehState->position.y, 0.05);

}
