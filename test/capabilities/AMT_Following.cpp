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
// Created by Jens Klimke on 2020-03-24.
// Contributors:
//
// AMT_Following.cpp
//


#include <simcore/functions.h>
#include <simcore/value/SignalTube.h>
#include "AgentModelAdapter.h"


/**
 * Test collection #8: Following
 * Here, the reaction is tested in following scenarios
 *
 * The scenarios are given in Gherkin syntax. The placeholder values are defined by parameters:
 *
 */
struct AMT_Following : public AgentModelAdapter {

    double initialVelocity = 20.0;
    double targetVelocity = 10.0;

    double xTarget0 = 1000.0;
    double xTarget1 =  800.0;

    double dsNet = INFINITY;

    agent_model::Target *target0 = nullptr;
    agent_model::Target *target1 = nullptr;

    AMT_Following() = default;
    ~AMT_Following() override = default;


    void setupTest(std::string &&label, unsigned int no) {

        // setup end conditions
        setup.endTime = 150.0;
        setup.endDistance = INFINITY;

        // create simulation
//        setup.logFile = sim::fnc::string_format("AMT_AccelerationControl_%s.json", label.c_str());
        setup.logTitle = sim::fnc::string_format("Test 8.%d", no);

        // create simulation
        create();

        // get target
        target0 = &drInput->targets[5];
        target1 = &drInput->targets[8];

        // init targets
        initTarget(target0, 1, xTarget0, targetVelocity);
        initTarget(target1, 2, xTarget1, targetVelocity);

        // set initial speed
        vehState->v = initialVelocity;

        if(EN_LOGGING) {

            // add target speed
            jsonLogger.addValue("vTar", &target0->v);
            plotLogger.trace("velocity", "vTar", "time", "vTar", "auto", 2);

            // add distance to log
            jsonLogger.addValue("ds", &dsNet);
            plotLogger.addFigure("distance", "Distance to following target", "time t [s]", "distance ds [m]", "time", "ds", false);

        }

    }

    void preStep(double simTime, double deltaTime) override {

        // set comfort speed
        drParam->velocity.vComfort = initialVelocity;

        // update targets
        updateTarget(target0, xTarget0, deltaTime);
        updateTarget(target1, xTarget1, deltaTime);

        // update net distance for relevant target
        dsNet = (xTarget1 - target1->size.length * 0.5) - (vehState->position.x + vehParam->size.x * 0.5);

        // run super pre step
        AgentModelAdapter::preStep(simTime, deltaTime);

    }


    static void initTarget(agent_model::Target *target, unsigned int id, double distance, double velocity) {

        // setup relevant target
        target->ds   = distance;
        target->v    = velocity;
        target->a    = 0.0;
        target->d    = 0.0;
        target->id   = id;
        target->lane = 0;
        target->psi  = 0.0;
        target->size = {5.0, 2.2};
        target->xy   = {0.0, 0.0};

    }


    void updateTarget(agent_model::Target *target, double &x, double deltaTime) {

        // set acceleration (do not increase 10 m/s)
        if(target->v >= 10.0 && target->a >= 0.0)
            target->a = 0.0;

        // target speed
        target->v = std::max(0.0, target->a * deltaTime + target->v);

        // new target position
        x += std::max(0.0, 0.5 * target->a * deltaTime * deltaTime + target->v * deltaTime);

        // target distance
        target->ds = x - (vehState->position.x + vehParam->driverPosition.x);

    }



};


/**
 * 1. Test: Target Following Control - Reach Distance
 *
 * Scenario:
 *  Given is a vehicle driver unit with a velocity of <init_speed>
 *   And a target with a velocity of <target_speed> and a initial distance of <init_dist>
 *   And the desired time headway is <time_headway>
 *   And the reaction time headway is <time_headway_max>
 *  When the time headway is larger than <time_headway_max> and the simulation ran for 3 seconds
 *   Then the acceleration shall be zero (+- 0.01)
 *  When the simulation ran for <sim_time> seconds
 *   Then the vehicle shall have reached a velocity of <target_speed>
 *   And the net distance to the target shall be <target_dist>.
 *
 * Values:
 *  sim_time = 30 s
 *  init_speed = 20 m/s
 *  target_speed = 10 m/s
 *  init_dist = 800 m
 *  target_dist = 10 m
 *  time_headway = 1.0 s
 *  time_headway_max = 15.0 s
 *
 */
TEST_F(AMT_Following, ReachDistance) {

    // setup test
    setupTest(test_info_->name(), 1);

    // set pre
    this->pre = [this] (double, double) {

        _param.follow.timeHeadway = 1.0;
        _param.follow.thwMax = 15.0;

    };

    // set post
    this->post = [this] (double simTime, double) {

        if(simTime >= 3.0 && drState->conscious.follow.distance > 200.0)
            EXPECT_NEAR(0.0, vehState->a, 0.01);
        else if(drState->conscious.follow.distance < 180 && drState->conscious.follow.distance > 15)
            EXPECT_LT(vehState->a, 0.0);

    };

    // run simulation
    run();

    // check final distance and speed
    EXPECT_NEAR(10.0, drState->conscious.follow.distance, 1e-3);
    EXPECT_NEAR(10.0, dsNet, 1e-3);

}


/**
 * 2. Test: Target Following Control - Reach Distance to Stopping Target
 *
 * Scenario:
 *  Given is a vehicle driver unit with a velocity of <init_speed>
 *   And a target with a velocity of <target_speed> and a initial distance of <init_dist>
 *   And the target decelerates continuously with <deceleration> to v = 0 from t = <t_start>
 *   And the desired time headway is <time_headway>
 *   And the desired stop distance is <stop_distance>
 *  When the simulation ran for <sim_time> seconds
 *   Then the vehicle shall have reached a velocity of zero
 *   And the net distance to the target shall be <stop_distance>.
 *
 * Values:
 *  sim_time = 30 s
 *  init_speed = 20 m/s
 *  target_speed = 10 m/s
 *  init_dist = 800 m
 *  deceleration = -0.5 m/s^2
 *  t_start = 30 s
 *  time_headway = 1.0 s
 *  stop_distance = 2.0 m
 *
 */
TEST_F(AMT_Following, ReachDistanceToStoppingTarget) {

    // setup test
    setupTest(test_info_->name(), 1);

    // set pre
    this->pre = [this] (double, double) {

        _param.follow.timeHeadway = 1.0;
        _param.follow.thwMax = 30.0;
        _param.follow.dsStopped = 2.0;

    };

    // set post
    this->post = [this] (double simTime, double) {

        // set deceleration
        if(simTime >= 10.0)
            target1->a = -0.2;

    };

    // run simulation
    run();

    // check final distance and speed
    EXPECT_NEAR(2.0, dsNet, 1e-1);

}


/**
 * 3. Test: Target Following Control - Stop and Go
 *
 * Scenario:
 *  Given is a vehicle driver unit with a velocity of <init_speed>
 *   And a target with a velocity of <target_speed> and a initial distance of <init_dist>
 *   And the target decelerates continuously with <deceleration> to v = 0  from t = <t0>
 *   And the target accelerates continuously with <acceleration> to v = 10 from t = <t1>
 *   And the desired time headway is <time_headway>
 *   And the desired stop distance is <stop_distance>
 *  When the simulation ran for <sim_time> seconds
 *   Then the vehicle shall have reached a velocity of <target_speed>
 *   And the net distance to the target shall be <target_dist>.
 *
 * Values:
 *  sim_time = 30 s
 *  init_speed = 20 m/s
 *  target_speed = 10 m/s
 *  init_dist = 800 m
 *  deceleration = -0.5 m/s^2
 *  accleraration = 0.5 m/s^2
 *  t0 = 5 s
 *  t1 =
 *  time_headway = 1.0 s
 *  stop_distance = 2.0 m
 *
 */
TEST_F(AMT_Following, StopAndGo) {

    // setup test
    setupTest(test_info_->name(), 1);

    // set pre
    this->pre = [this] (double, double) {

        _param.follow.timeHeadway = 1.0;
        _param.follow.dsStopped = 2.0;

    };

    // set post
    this->post = [this] (double simTime, double) {

        // set deceleration
        if(simTime >= 10.0)
            target1->a = -0.2;

        if(simTime >= 80.0)
            target1->a = 0.2;

    };

    // run simulation
    run();

    // check final distance and speed
    EXPECT_NEAR(10.0, drState->conscious.follow.distance, 1e-3);
    EXPECT_NEAR(10.0, dsNet, 1e-3);

}
