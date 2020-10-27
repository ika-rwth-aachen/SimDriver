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
// Created by Jens Klimke on 2020-03-26.
// Contributors:
//
// AMT_SpeedControl.cpp
//


#include <simcore/functions.h>
#include <simcore/value/SignalTube.h>
#include "AgentModelAdapter.h"


/**
 * Test collection #9: Speed control
 * Here, the reaction is tested in speed scenarios
 * 1. comfort speed
 * 2. rules
 * 3. curves
 *
 * The scenarios are given in Gherkin syntax. The placeholder values are defined by parameters:
 *
 */
struct AMT_SpeedControl : public AgentModelAdapter {

    double simulationTime = 30.0;
    double simulationDistance = INFINITY;
    double initialVelocity = 20.0;
    double ay = 0.0;
    double kappa = 0.0;

    double v0 = 0.0;
    double s0 = 0.0;

    AMT_SpeedControl() = default;
    ~AMT_SpeedControl() override = default;


    void setupTest(std::string &&label, unsigned int no) {

        // setup end conditions
        setup.endTime = simulationTime;
        setup.endDistance = simulationDistance;

        // create simulation
//        setup.logFile = sim::fnc::string_format("AMT_AccelerationControl_%s.json", label.c_str());
        setup.logTitle = sim::fnc::string_format("Test 8.%d", no);

        // create simulation
        create();

        // set initial speed
        vehState->v = initialVelocity;
        drParam->velocity.vComfort = initialVelocity;

        if(EN_LOGGING) {

            jsonLogger.addValue("v0", &v0);
            jsonLogger.addValue("s0", &s0);

            // add target speed
            plotLogger.addFigure("speedlimit", "Speed limits", "Distance s [m]", "Velocity v [m/s]", "s", {
                {"Act. velocity", "v"},
                {"Rule velocity", "vRule"},
                {"Curve velocity", "vCurve"}
            });

            // add trace
            plotLogger.trace("speedlimit", "internal velocity", "s0", "v0", "auto", 2);

            // add lateral acceleration
            jsonLogger.addValue("ayTest", &ay);
            plotLogger.trace("acceleration", "lateral acc.", "time", "ayTest", "auto", 2);

        }

    }

    void preStep(double simTime, double deltaTime) override {

        // run super pre step
        AgentModelAdapter::preStep(simTime, deltaTime);

    }


    void postStep(double simTime, double deltaTime) override {

        // run super pre step
        AgentModelAdapter::postStep(simTime, deltaTime);

        s0 = vehState->s;
        v0 = _vel_horizon.mean(s0, s0 + vehState->v * 15.0);

    }


};


/**
 * 1. Test: Speed control - Comfort speed
 *
 * Scenario:
 *  Given is a vehicle driver unit with a velocity of <init_speed>
 *   And the driver has a comfort speed of <init_speed>
 *   And when t = 10 the comfort speed is set to <target_speed>
 *  When the simulation ran for <sim_time> seconds
 *   Then the vehicle shall have reached a velocity of <target_speed>
 *
 * Values:
 *  sim_time = 30 s
 *  init_speed = 20 m/s
 *  target_speed = 10 m/s
 *
 */
TEST_F(AMT_SpeedControl, ComfortSpeed) {

    // setup test
    setupTest(test_info_->name(), 1);

    // set pre
    this->pre = [this] (double simTime, double) {

        if(simTime > 9.999 && simTime < 10.001)
            drParam->velocity.vComfort = 10.0;

    };

    // set post
    this->post = [this] (double simTime, double) {
    };

    // run simulation
    run();

    EXPECT_NEAR(10.0, vehState->v, 1e-3);

}


/**
 * 2. Test: Speed control - Comfort speed cosine
 *
 * Scenario:
 *  Given is a vehicle driver unit with a velocity of <init_speed>
 *   And the driver has a comfort speed of <init_speed>
 *   And the driver's comfort speed oscillates cosine shaped with an amplitude of <amp> and a frequency of <freq>
 *  When a simulation time step has be performed
 *   Then actual speed shall be close to the comfort speed (+- 0.1 m/s)
 *
 * Values:
 *  sim_time = 30 s
 *  init_speed = 20 m/s
 *  amp = 5 m/s
 *  freq = 0.05 1/s
 *  target_speed = 10 m/s
 *
 */
TEST_F(AMT_SpeedControl, ComfortSpeedCosine) {

    // setup test
    simulationTime = 60.0;
    setupTest(test_info_->name(), 2);

    // set pre
    this->pre = [this] (double simTime, double) {

        drParam->velocity.vComfort = 2.5 * cos(simTime * 0.05 * 2.0 * M_PI) + 17.5;

    };

    // set post
    this->post = [this] (double simTime, double) {

        EXPECT_NEAR(drParam->velocity.vComfort, vehState->v, 2.0);

    };

    // run simulation
    run();

}


/**
 * 3. Test: Speed control - Speed Rule
 *
 * Scenario:
 *  Given is a vehicle driver unit with a velocity of <init_speed>
 *   And the track has a speed limit of 70 kph at 500 m
 *   And the track has a speed limit of 50 kph at 1000 m
 *   And the track has a speed limit of 100 kph at 1500 m
 *   And the track has a speed limit of 50 kph at 1700 m
 *   And the track has a speed limit of 30 kph at 1800 m
 *   And the speed limits are unset at 2000 m
 *  When a simulation time step has be performed
 *   Then actual speed shall be close to the comfort speed (+- 0.1 m/s)
 *
 * Values:
 *  sim_time = 30 s
 *  init_speed = 20 m/s
 *  amp = 5 m/s
 *  freq = 0.05 1/s
 *  target_speed = 10 m/s
 *
 */
TEST_F(AMT_SpeedControl, Rules) {

    // setup test
    simulationTime = 320.0;
    simulationDistance = 3500.0;
    initialVelocity = 120.0 / 3.6;

    setupTest(test_info_->name(), 3);

    // set pre
    this->pre = [this] (double simTime, double) {

        // TODO: different order

        drInput->signals[0].id = 1;
        drInput->signals[0].ds = 500.0 - vehState->s;
        drInput->signals[0].type = agent_model::SIGNAL_SPEED_LIMIT;
        drInput->signals[0].value = 70;

        drInput->signals[1].id = 2;
        drInput->signals[1].ds = 1000.0 - vehState->s;
        drInput->signals[1].type = agent_model::SIGNAL_SPEED_LIMIT;
        drInput->signals[1].value = 50;

        drInput->signals[2].id = 3;
        drInput->signals[2].ds = 1500.0 - vehState->s;
        drInput->signals[2].type = agent_model::SIGNAL_SPEED_LIMIT;
        drInput->signals[2].value = 100;

        drInput->signals[3].id = 3;
        drInput->signals[3].ds = 1700.0 - vehState->s;
        drInput->signals[3].type = agent_model::SIGNAL_SPEED_LIMIT;
        drInput->signals[3].value = 50;

        drInput->signals[4].id = 4;
        drInput->signals[4].ds = 1800.0 - vehState->s;
        drInput->signals[4].type = agent_model::SIGNAL_SPEED_LIMIT;
        drInput->signals[4].value = 30;

        drInput->signals[5].id = 5;
        drInput->signals[5].ds = 2000.0 - vehState->s;
        drInput->signals[5].type = agent_model::SIGNAL_SPEED_LIMIT;
        drInput->signals[5].value = -1;


    };

    // set post
    this->post = [this] (double simTime, double) {

//        if(vehState->s > 650.0 && vehState->s < 800.0)
//            EXPECT_NEAR(70.0 / 3.6, vehState->v, 0.1);
//        else if(vehState->s > 1100.0 && vehState->s < 1500.0)
//            EXPECT_NEAR(50.0 / 3.6, vehState->v, 0.1);
//        else if(vehState->s > 1500.0 && vehState->s < 1700.0)
//            EXPECT_GE(100.0 / 3.6, vehState->v);
//        else if(vehState->s > 1710.0 && vehState->s < 1800.0)
//            EXPECT_GE(50.0 / 3.6, vehState->v);
//        else if(vehState->s > 1850.0 && vehState->s < 2000.0)
//            EXPECT_NEAR(30.0 / 3.6, vehState->v, 0.1);

    };

    // run simulation
    run();

    // check spee´´01 EXPECT_NEAR(120.0 / 3.6, vehState->v, 0.1);

}


/**
 * 4. Test: Speed control - Curve speed
 *
 * Scenario:
 *  Given is a vehicle driver unit with a velocity of <init_speed>
 *   And the track has a curve with a curvature of 0.01 1/m between 500 m and 1000 m
 *   And the track has a curve with a curvature of -0.01 1/m between 1200 m and 1500 m
 *   And the track has a curve with a curvature of 0.1 1/m between 1800 m and 10000 m
 *   And the track has a curve with a curvature of 0.01 1/m between 2500 m and 2600 m
 *   And the track has a curve with a curvature of 0.05 1/m between 2600 m and 2700 m
 *  When the simulation ran for <sim_time> seconds
 *   Then the vehicle shall have reached a velocity of <target_speed>
 *
 * Values:
 *  sim_time = 30 s
 *  init_speed = 20 m/s
 *  target_speed = 10 m/s
 *
 */
TEST_F(AMT_SpeedControl, CurveSpeed) {

    // TODO: check t = 200.3 - 200.6 -> acceleration steps

    // setup test
    simulationTime = 320.0;
    simulationDistance = 3500.0;
    initialVelocity = 120.0 / 3.6;

    // setup test
    setupTest(test_info_->name(), 1);

    // set pre
    this->pre = [this] (double simTime, double) {

        for(unsigned int i = 0; i < agent_model::NOH; ++i) {

            double ds = std::pow(0.8 * (double) i, 2.0);
            double s = ds + vehState->s;

            double k = 0.0;
            if(s > 500.0 && s < 800.0)
                k = 0.01;
            else if(s > 1200.0 && s < 1500.0)
                k = -0.01;
            else if(s > 1800.0 && s < 2000.0)
                k = 0.1;
            else if(s > 2500.0 && s < 2600.0)
                k = 0.01;
            else if(s > 2600.0 && s < 2700.0)
                k = 0.05;

            _input.horizon.ds[i] = ds;
            _input.horizon.kappa[i] = k;

            if(i == 0) {
                ay = k * vehState->v * vehState->v;
                kappa = k;
            }

        }

    };

    // set post
    this->post = [this] (double simTime, double) {

    };

    // run simulation
    run();

}