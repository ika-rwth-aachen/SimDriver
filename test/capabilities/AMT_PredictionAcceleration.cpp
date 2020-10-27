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


#include <simcore/functions.h>
#include <simcore/value/SignalTube.h>
#include <cmath>
#include "AgentModelAdapter.h"
#include <model_collection.h>


/**
 * Test collection #3: Acceleration due to predictive behavior
 * Here, the acceleration reaction is tested.
 *
 * The scenarios are given in Gherkin syntax. The placeholder values are defined by parameters:
 *
 */
struct AMT_PredictionAcceleration : public AgentModelAdapter, public SignalTube {

    double endTime = 60.0;
    double endDistance = 400.0;

    double initialVelocity = 20.0;
    double followVelocity = 10.0;

    double followDistance = 300.0;
    double targetPosition = 300.0;
    double ds = INFINITY;

    struct {
        std::vector<double> lx{};
        std::vector<double> ly{};
        std::vector<double> ux{};
        std::vector<double> uy{};
        std::string x{};
    } tube;

    double *v = nullptr;
    double *s = nullptr;

    AMT_PredictionAcceleration() = default;
    ~AMT_PredictionAcceleration() override = default;


    void setupTest(std::string &&label, unsigned int no) {

        // setup end conditions
        setup.endTime = endTime;
        setup.endDistance = endDistance;

        // create simulation
//        setup.agents.push_back({initialVelocity, {"R1-LS1-R1", 0.0, 0.0, {"1"}}});
//        setup.map = sim::fnc::string_format("%s/Straight10000.xodr", MAP_DIR);
//        setup.logFile = sim::fnc::string_format("AMT_PredictionAccelerationControl_%s.json", label.c_str());
        setup.logTitle = sim::fnc::string_format("Test 3.%d", no);

        // create simulation
        create();

        // set initial pedal value, to avoid controller to settle too long
        vehInput->pedal = 0.1;
        vehState->v = initialVelocity;
        drParam->velocity.vComfort = initialVelocity;

        // set v and s
        v = &vehState->v;
        s = &vehState->s;

        if(EN_LOGGING) {

            // add distance to log
            jsonLogger.addValue("ds", &ds);

            // add to figure
            plotLogger.addFigure("distance", "Distance", "time t [s]", "distance ds [m]", "time", "ds");

            // add to figure
            plotLogger.addFigure("velocity_over_s", "Velocity over position", "long. position [m]",
                                 "velocity [m/s]", "s", "v");

            if(tube.x == "s") {

                // add bounds for visualization
                plotLogger.trace("velocity_over_s", "lower bound", tube.lx, tube.ly, "rgb(191, 191, 191)", 3);
                plotLogger.trace("velocity_over_s", "upper bound", tube.ux, tube.uy, "rgb(191, 191, 191)", 3);

            }

        }

        // add figure
        if(tube.x == "s") {

            // set value tube
            defineUpper(std::move(std::vector<double>(tube.ux)), std::move(std::vector<double>(tube.uy)));
            defineLower(std::move(std::vector<double>(tube.lx)), std::move(std::vector<double>(tube.ly)));

        }

    }


    void preStep(double simTime, double deltaTime) override {

        // save distance
        followDistance += followVelocity * deltaTime;
        ds = targetPosition - vehState->s;

        // run pre step of GenericTest
        AgentModelAdapter::preStep(simTime, deltaTime);

    }


};


/**
 * 1. Test: Acceleration Control - Speed Prediction
 *
 * Scenario:
 *  Given is a vehicle driver unit
 *   And the vehicle is travelling with an initial velocity of <init_velocity>
 *   And the driver having a comfort target velocity of <init_velocity>
 *   And the target speed at the position <target_position> shall be <target_velocity>
 *  When simulation ran for <sim_time> seconds
 *   Then the vehicle shall have reached a velocity of <target_velocity>
 *   And the acceleration shall be almost zero.
 *
 * Values:
 *  sim_time = 30 s
 *  init_velocity = 20.0 m/s
 *  target_position = 300.0 m
 *  target_velocity = 10.0 (+- 1 m/s)
 */
TEST_F(AMT_PredictionAcceleration, SpeedPrediction) {

    // set pre
    this->pre = [this] (double, double) {

        double targetVelocity = 10.0;
        double dsMax = vehState->v * drParam->velocity.thwMax;

        // get factors
        double f0 = agent_model::scale(ds, dsMax, 0.0, 2.0); // std::max(0.0, std::min(1.0, ds / dsMax));
        double f1 = (1.0 - f0);

        // inject speed
        conscious->velocity.prediction = f0 * initialVelocity + f1 * targetVelocity;
        conscious->velocity.local = ds <= 0.0 ? targetVelocity : initialVelocity;

    };

    // set post
    this->post = [this] (double simTime, double) {

        EXPECT_LE(-_param.velocity.a, vehState->a);
        EXPECT_TRUE(in(*s, *v));

    };

    // define bounds
    tube.ux = {0.0, 200.0, 300.0, 400.0};
    tube.uy = {20.1, 20.1,  10.1,  10.1};
    tube.lx = {0.0, 100.0, 200.0, 400.0};
    tube.ly = {19.9, 19.9,   9.9,   9.9};
    tube.x = "s";

    // setup test
    setupTest(test_info_->name(), 1);

    // run simulation
    run();

    // check pedal value
    ASSERT_NEAR(10.0, vehState->v, 1.0);
    ASSERT_NEAR(0.0, vehState->a, 0.1);

}


/**
 * 2. Test: Acceleration Control - Speed Prediction Curve
 *
 * Scenario:
 *  Given is a vehicle driver unit
 *   And the vehicle is travelling with an initial velocity of <init_velocity>
 *   And the driver having a comfort target velocity of <init_velocity>
 *   And the target speed at the position <target_position> shall be <target_velocity>
 *  When simulation ran for <sim_time> seconds
 *   Then the vehicle shall have reached a velocity of <target_velocity>
 *   And the acceleration shall be almost zero.
 *
 * Values:
 *  sim_time = 30 s
 *  init_velocity = 20.0 m/s
 *  target_position = 300.0 m
 *  target_velocity = 10.0 (+- 1 m/s)
 */
TEST_F(AMT_PredictionAcceleration, SpeedPredictionCurve) {

    endDistance = 700.0;

    // set pre
    this->pre = [this] (double, double) {

        double s0 = 300.0;
        double s1 = 325.0;
        double s2 = 375.0;
        double s3 = 400.0;

        double k0 = 0.0;
        double k1 = 0.01;
        double k2 = 0.01;
        double k3 = 0.0;

        for(unsigned i = 0; i < agent_model::NOH; ++i) {

            auto ds = i * (drParam->velocity.thwMax * drInput->vehicle.v) / (agent_model::NOH - 1);
            auto s = vehState->s - drParam->vehicle.pos.x + ds;

            double kappa = 0.0;
            if(s >= s0 && s <= s1) {
                kappa = (s - s0) * (k1 - k0) / (s1 - s0) + k0;
            } else if(s > s1 && s < s2) {
                kappa = (s - s1) * (k2 - k1) / (s2 - s1) + k1;
            } else if(s >= s2 && s <= s3) {
                kappa = (s - s2) * (k3 - k2) / (s3 - s2) + k2;
            }

            // set horizon
            drInput->horizon.ds[i]    = ds;
            drInput->horizon.kappa[i] = kappa;
            drInput->horizon.x[i]     = ds;
            drInput->horizon.y[i]     = -drParam->vehicle.pos.y;

        }

    };

    // set post
    this->post = [this] (double simTime, double) {

        EXPECT_LE(-_param.velocity.a, vehState->a);
        EXPECT_TRUE(in(*s, *v));

    };

    // define bounds
    tube.ux = {0.0, 200.0, 320.0, 380.0};
    tube.uy = {20.1, 20.1,  11.6,  11.6};
    tube.lx = {0.0, 100.0, 200.0, 400.0};
    tube.ly = {19.9, 19.9,  11.2,  11.2};
    tube.x = "s";

    // setup test
    setupTest(test_info_->name(), 3);

    // run simulation
    run();

    // check pedal value
    ASSERT_NEAR(20.0, vehState->v, 1.0);
    ASSERT_NEAR(0.0, vehState->a, 0.1);

}


/**
 * 2b. Test: Acceleration Control - Speed Prediction Rule
 *
 * Scenario:
 *  Given is a vehicle driver unit
 *   And the vehicle is travelling with an initial velocity of <init_velocity>
 *   And the driver having a comfort target velocity of <init_velocity>
 *   And the target speed at the position <target_position> shall be <target_velocity>
 *  When simulation ran for <sim_time> seconds
 *   Then the vehicle shall have reached a velocity of <target_velocity>
 *   And the acceleration shall be almost zero.
 *
 * Values:
 *  sim_time = 30 s
 *  init_velocity = 20.0 m/s
 *  target_position = 300.0 m
 *  target_velocity = 10.0 (+- 1 m/s)
 */
TEST_F(AMT_PredictionAcceleration, SpeedPredictionRule) {

    // set pre
    this->pre = [this] (double, double) {

        double targetVelocity = 10.0;

        _input.signals[0].ds = -vehState->s - 1.0;
        _input.signals[0].id = 1;
        _input.signals[0].type = agent_model::SignalType::SIGNAL_SPEED_LIMIT;
        _input.signals[0].value = floor(3.6 * initialVelocity);

        _input.signals[1].ds = ds;
        _input.signals[1].id = 2;
        _input.signals[1].type = agent_model::SignalType::SIGNAL_SPEED_LIMIT;
        _input.signals[1].value = floor(3.6 * targetVelocity);

    };

    // set post
    this->post = [this] (double simTime, double) {

        EXPECT_LE(-_param.velocity.a, vehState->a);
        EXPECT_TRUE(in(*s, *v));

    };

    // define bounds
    tube.ux = {0.0, 200.0, 300.0, 400.0};
    tube.uy = {20.1, 20.1,  10.1,  10.1};
    tube.lx = {0.0, 100.0, 200.0, 400.0};
    tube.ly = {19.9, 19.9,   9.9,   9.9};
    tube.x = "s";

    // setup test
    setupTest(test_info_->name(), 2);

    // run simulation
    run();

    // check pedal value
    ASSERT_NEAR(10.0, vehState->v, 1.0);
    ASSERT_NEAR(0.0, vehState->a, 0.1);

}


/**
 * 3a. Test: Acceleration Control - Stopping
 *
 * Scenario:
 *  Given is a vehicle driver unit
 *  And the vehicle is travelling with an initial velocity of <init_velocity>
 *  And the driver having a comfort target velocity of <init_velocity>
 *  And the target stop is located at the position <target_position>
 *  When simulation ran for <sim_time> seconds
 *  Then the vehicle shall have stopped
 *  And the acceleration shall be almost zero
 *  And the travelled distance shall be almost <target_position>.
 *
 * Values:
 *  sim_time = 30 s
 *  init_velocity = 20.0 m/s
 *  target_position = 300.0 m (+- 0.01 m)
 */
TEST_F(AMT_PredictionAcceleration, Stopping) {

    // set end time
    endTime = 40.0;

    // set pre
    this->pre = [this] (double, double) {

        // injection of prediction
        conscious->stop.ds = ds;
        conscious->stop.dsMax = 200.0;

        // injection of the comfort speed
        drParam->velocity.vComfort = initialVelocity;

    };

    // set post
    this->post = [this] (double simTime, double) {

        // EXPECT_LE(-_param.velocity.a, vehState->a);
        EXPECT_TRUE(in(*s, *v));

    };

    // define bounds
    tube.lx = {0.0, 100.0, 300.0, 400.0};
    tube.ly = {19.9, 19.9,  -0.1,  -0.1};
    tube.ux = {0.0, 300.0, 300.1, 400.0};
    tube.uy = {20.1, 20.1,   0.1,   0.1};
    tube.x = "s";

    // setup test
    setupTest(test_info_->name(), 3);

    // run simulation
    run();

    // check pedal value
    ASSERT_NEAR(0.0, vehState->v, 0.01);
    ASSERT_NEAR(0.0, vehState->a, 0.1);
    ASSERT_NEAR(targetPosition, vehState->s, 0.01);

}


/**
 * 3b. Test: Acceleration Control - Stopping (process)
 *
 * Scenario:
 *  Given is a vehicle driver unit
 *  And the vehicle is travelling with an initial velocity of <init_velocity>
 *  And the driver having a comfort target velocity of <init_velocity>
 *  And the target stop is located at the position <target_position>
 *  When simulation ran for <sim_time> seconds
 *  Then the vehicle shall have stopped
 *  And the acceleration shall be almost zero
 *  And the travelled distance shall be almost <target_position>.
 *
 * Values:
 *  sim_time = 30 s
 *  init_velocity = 20.0 m/s
 *  target_position = 300.0 m (+- 0.3 m)
 */
TEST_F(AMT_PredictionAcceleration, StoppingWithProcess) {

    // set end time
    endTime = 40.0;

    // set pre
    this->pre = [this] (double simTime, double) {

        // decision injection
        if(simTime > 4.9999 && simTime < 5.0001) {

            decisions->stopping[0].id = 1;
            decisions->stopping[0].position = targetPosition;
            decisions->stopping[0].standingTime = INFINITY;

        }

        // injection of the comfort speed
        drParam->velocity.vComfort = initialVelocity;

    };

    // set post
    this->post = [this] (double simTime, double) {

        // EXPECT_LE(-_param.velocity.a, vehState->a);
        EXPECT_TRUE(in(*s, *v));

    };

    // define bounds
    tube.lx = {0.0, 100.0, 300.0, 400.0};
    tube.ly = {19.9, 19.9,  -0.1,  -0.1};
    tube.ux = {0.0, 300.0, 300.1, 400.0};
    tube.uy = {20.1, 20.1,   0.1,   0.1};
    tube.x = "s";

    // setup test
    setupTest(test_info_->name(), 3);

    // run simulation
    run();

    // check pedal value
    ASSERT_NEAR(0.0, vehState->v, 0.01);
    ASSERT_NEAR(0.0, vehState->a, 0.1);
    ASSERT_NEAR(targetPosition, vehState->s, 0.3);
    ASSERT_NEAR(_param.stop.pedalDuringStanding, vehInput->pedal, 1e-6);

}


/**
 * 3c. Test: Acceleration Control - Stopping -> continue -> stopping
 *
 * Scenario:
 *  Given is a vehicle driver unit
 *  And the vehicle is travelling with an initial velocity of <init_velocity>
 *  And the driver having a comfort target velocity of <init_velocity>
 *  And the target stop is located at the position <target_position1>
 *  And another target stop is located at the position <target_position2>
 *  And the first stop is deactivated <time>
 *  When the first stop is reached
 *  Then the vehicle shall have stopped
 *  And the acceleration shall be almost zero
 *  And the travelled distance shall be almost <target_position>.
 *  When simulation ran for <sim_time> seconds
 *  Then the vehicle shall have stopped
 *  And the acceleration shall be almost zero
 *  And the travelled distance shall be almost <target_position2>.
 *
 * Values:
 *  sim_time = 30 s
 *  init_velocity = 20.0 m/s
 *  target_position = 300.0 m (+- 0.3 m)
 *  target_position2 = 500.0 m (+- 0.3 m)
 *  time = 35 s
 */
TEST_F(AMT_PredictionAcceleration, StoppingContinueStopping) {

    // set end time
    endTime = 100.0;
    endDistance = 1000.0;

    // set pre
    this->pre = [this] (double simTime, double) {

        // decision injection
        if(simTime > 4.9999 && simTime < 5.0001) {

            decisions->stopping[0].id = 1;
            decisions->stopping[0].position = targetPosition;
            decisions->stopping[0].standingTime = 1.0;

            decisions->stopping[1].id = 2;
            decisions->stopping[1].position = targetPosition + 200.0;
            decisions->stopping[1].standingTime = INFINITY;

        }

        // injection of the comfort speed
        drParam->velocity.vComfort = initialVelocity;

    };

    // set post
    this->post = [this] (double simTime, double) {

        // EXPECT_LE(-_param.velocity.a, vehState->a);
        EXPECT_TRUE(in(*s, *v));

    };

    // define bounds
    tube.lx = {0.0, 100.0, 300.0, 500.0};
    tube.ly = {19.9, 19.9,  -0.1,  -0.1};
    tube.ux = {0.0, 290.0, 300.0, 310.0, 490.0, 500.0};
    tube.uy = {20.1, 20.1,   2.0,  20.1,  20.1,   0.1};
    tube.x = "s";

    // setup test
    setupTest(test_info_->name(), 3);

    // run simulation
    run();

    // check pedal value
    ASSERT_NEAR(0.0, vehState->v, 0.01);
    ASSERT_NEAR(0.0, vehState->a, 0.1);
    ASSERT_NEAR(targetPosition + 200.0, vehState->s, 0.3);
    ASSERT_NEAR(_param.stop.pedalDuringStanding, vehInput->pedal, 1e-6);

}



/**
 * 4. Test: Acceleration Control - Following
 *
 * Scenario:
 *  Given is a vehicle driver unit
 *  And the vehicle is travelling with an initial velocity of <init_velocity>
 *  And the driver having a comfort target velocity of <init_velocity>
 *  And the object to be followed is at the initial position <init_follow_position>
 *  And the object to be followed has a constant velocity of <follow_velocity>
 *  When simulation ran for <sim_time> seconds
 *  Then the vehicle shall have a velocity of <follow_velocity>
 *  And the acceleration shall be almost zero
 *  And the distance to the follow object shall be <follow_distance>
 *
 * Values:
 *  sim_time = 60 s
 *  init_velocity = 20.0 m/s
 *  init_follow_position = 300.0 m
 *  follow_velocity = 10.0 m/s
 *  follow_distance = 18.0 m
 *
 */
TEST_F(AMT_PredictionAcceleration, Following) {

    // set end condition
    endDistance = 1000.0;
    endTime = 60.0;

    // set pre
    this->pre = [this] (double, double) {

        // set ds to follow distance
        ds = followDistance - vehState->s;

        // injection of prediction
        conscious->follow.distance = ds;
        conscious->follow.velocity = followVelocity;

        // injection of the comfort speed
        drParam->velocity.vComfort = initialVelocity;

    };

    // setup test
    setupTest(test_info_->name(), 4);

    // run simulation
    run();

    // check pedal value
    ASSERT_NEAR(followVelocity, vehState->v, 1e-3);
    ASSERT_NEAR( 0.0, vehState->a, 0.1);
    ASSERT_NEAR(15.0, ds, 0.01);

}


/**
 * 5. Test: Acceleration Control - Following with non-constant speed
 *
 * Scenario:
 *  Given is a vehicle driver unit
 *  And the vehicle is travelling with an initial velocity of <init_velocity>
 *  And the driver having a comfort target velocity of <init_velocity>
 *  And the driver having a desired time headway of <time_headway>
 *  And the object to be followed is at the initial position <init_follow_position>
 *  And the object to be followed has an initial velocity of <follow_velocity>
 *  And the velocity of the object to be followed is oscillating with +- <ampl> and a frequency of <freq> around the initial velocity
 *  When simulation runs
 *  Then the time headway shall be <time_headway>
 *
 * Values:
 *  init_velocity = 20.0 m/s
 *  init_follow_position = 300.0 m
 *  time_headway = 1.8 s
 *  follow_velocity = 10.0 m/s
 *  follow_distance = 18.0 m
 *  ampl = 1.0 m/s
 *  freq = 0.1 1/s
 *
 */
TEST_F(AMT_PredictionAcceleration, FollowingWithNonConstantSpeed) {

    // set end condition
    endDistance = 1000.0;
    endTime = 60.0;

    // set pre
    this->pre = [this] (double, double) {

        // set ds to follow distance
        ds = followDistance - vehState->s;

        // injection of prediction
        conscious->follow.distance = ds;
        conscious->follow.velocity = followVelocity;

        // injection of the comfort speed
        drParam->velocity.vComfort = initialVelocity;

    };

    this->post = [this] (double simTime, double) {

        if(simTime > 40.0) {

            // check pedal value
            EXPECT_NEAR(followVelocity, vehState->v, 0.8);
            EXPECT_NEAR(1.5, ds / followVelocity, 0.2);

        }

        // set follow velocity
        followVelocity = 10.0 + 1.0 * sin(2.0 * M_PI * simTime / 10.0);

    };

    // setup test
    setupTest(test_info_->name(), 5);

    // run simulation
    run();

}
