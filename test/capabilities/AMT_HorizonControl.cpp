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
// Created by Jens Klimke on 2020-03-16.
// Contributors:
//
// AMT_HorizonControl.cpp
//

#include <simcore/functions.h>
#include "AgentModelAdapter.h"

/**
 * Test collection #7: Horizon control
 * Here, the steering reaction is tested.
 */
struct AMT_HorizonControl : public AgentModelAdapter {

    struct {
        double x;
        double y;
    } pos{};

    double radius = 500.0;
    double endTime = 30.0;

    double factor0 = 0.0;
    double factor1 = 0.0;
    double factor2 = 0.0;

    double theta0 = 0.0;
    double theta1 = 0.0;
    double theta2 = 0.0;
    double thetaR = 0.0;

    AMT_HorizonControl() = default;
    ~AMT_HorizonControl() override = default;

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
        setup.logTitle = sim::fnc::string_format("Test 7.%d", no);

        // create simulation
        create();

        if(EN_LOGGING) {

            // add logger
            jsonLogger.addValue("x0", &pos.x);
            jsonLogger.addValue("y0", &pos.y);
            jsonLogger.addValue("f0", &factor0);
            jsonLogger.addValue("f1", &factor1);
            jsonLogger.addValue("f2", &factor2);
            jsonLogger.addValue("th0", &theta0);
            jsonLogger.addValue("th1", &theta1);
            jsonLogger.addValue("th2", &theta2);
            jsonLogger.addValue("thR", &thetaR);

            // add trace
            plotLogger.trace("position", "ref", "x0", "y0", "auto", 2);

            // add to figure
            plotLogger.addFigure("yawRate", "Yaw rate", "time t [s]", "yaw rate [1/s]", "time", "dPsi");
            plotLogger.addFigure("offset_over_t", "Offset over time", "time t [s]", "offset [m]", "time", "y");
            plotLogger.addFigure("offset_over_s", "Offset over position", "long. position [m]", "offset [m]", "s", "y");

            // add factors figure
            plotLogger.addFigure("factors", "Factors", "time t [s]", "factor [-]", "time",
                    {{"factor 0", "f0"}, {"factor 1", "f1"}, {"factor 2", "f2"}});

            // add factors figure
            plotLogger.addFigure("theta", "Reference point angles", "time t [s]", "theta [-]", "time",
                                 {{"theta 0", "th0"}, {"theta 1", "th1"}, {"theta 2", "th2"}, {"theta ref.", "thR"}});

        }

        // set vehicle speed
        vehState->position.x = 0.0;
        vehState->position.y = 0.0;
        vehState->psi = 0.0;
        vehState->v = 10.0;
        vehState->psi = 0.0;
        vehInput->steer = 0.0;

    }


    void preStep(double simTime, double deltaTime) override {

        // set comfort speed
        drParam->velocity.vComfort = 10.0;

        // get reference value
        factor0 = drState->conscious.lateral.paths[0].factor;
        factor1 = drState->conscious.lateral.paths[1].factor;
        factor2 = drState->conscious.lateral.paths[2].factor;

        // get thetas
        theta0 = drState->aux[0];
        theta1 = drState->aux[2];
        theta2 = drState->aux[4];
        thetaR = drState->aux[31];

        // execute pre
        AgentModelAdapter::preStep(simTime, deltaTime);

    }


    void calculateHorizon(double s0, double x, double y, double psi, unsigned int i, double sMax = INFINITY) {

        // save position
        if(i == 0) {
            pos.x = vehState->position.x + x;
            pos.y = vehState->position.y + y;
        }

        // rotation
        double c = cos(-vehState->psi);
        double s = sin(-vehState->psi);

        // calculate point
        double x0 = c * x - s * y - vehParam->driverPosition.x;
        double y0 = s * x + c * y - vehParam->driverPosition.y;
        double psi0 = psi - vehState->psi;

        if(s0 > sMax) {
            s0 = INFINITY;
            x0 = INFINITY;
            y0 = 10.0; // just to set a stupid value
            psi0 = M_PI; // just to set a stupid value
        }

        // set horizon points
        _input.horizon.ds[i] = s0;
        _input.horizon.x[i] = x0;
        _input.horizon.y[i] = y0;
        _input.horizon.psi[i] = psi0;
        _input.horizon.kappa[i] = 0.0;
        _input.horizon.egoLaneWidth[i] = 3.75;
        _input.horizon.rightLaneWidth[i] = 3.75;
        _input.horizon.leftLaneWidth[i] = 3.75;

    }


};


/**
 * 1. Test: Horizon Control - Compensating a lateral offset
 *
 * Scenario:
 *  Given is a vehicle driver unit with a velocity of <init_speed>
 *  And a straight track (x=0, y=0, phi=0)
 *  And the vehicle has an initial offset of <init_offset>
 *  When the simulation time of <sim_time> has passed
 *  Then the y-offset of the vehicle shall be zero (1e-3)
 *  And the vehicle shall move in x-direction (1e-3)
 *
 * Values:
 *  sim_time = 7 s
 *  init_speed = 10 m/s
 *  init_offset = 2 m
 */
TEST_F(AMT_HorizonControl, CompensatingOffset) {

    endTime = 7.0;
    double trackLen = 200.0;

    // setup simulation
    setupTest(test_info_->name(), 2);

    // set pre
    this->pre = [this, trackLen] (double simTime, double deltaTime) {

        // set lateral offset to track
        if(vehState->position.x > trackLen)
            _input.vehicle.d = 300.0; // just to set to a stupid value

        // set horizon points
        for(unsigned int i = 0; i < agent_model::NOH; ++i) {

            // subtract ego position
            double x = std::pow(0.5 * i, 2.0);
            double y = -vehState->position.y;

            // distance to end
            double sEnd = trackLen - vehState->position.x;

            // calculate horizon
            calculateHorizon(x, x, y, 0.0, i, sEnd);

        }

    };

    // set vehicle position
    vehState->position.x = 0.0;
    vehState->position.y = 2.0;

    // run simulation
    run();

    EXPECT_NEAR(0.0, vehState->position.y, 1e-3);
    EXPECT_NEAR(0.0, vehState->psi, 1e-3);

}


/**
 * 1b. Test: Horizon Control - Compensating a lateral offset
 *
 * Scenario:
 *  Given is a vehicle driver unit with a velocity of <init_speed>
 *  And a straight track (x=0, y=0, phi=0.5*pi)
 *  And the vehicle has an initial offset of <init_offset>
 *  When the simulation time of <sim_time> has passed
 *  Then the x-offset of the vehicle shall be zero (1e-3)
 *  And the vehicle shall move in y-direction (1e-3)
 *
 * Values:
 *  sim_time = 7 s
 *  init_speed = 10 m/s
 *  init_offset = 2 m
 */
TEST_F(AMT_HorizonControl, CompensatingOffsetY) {

    endTime = 7.0;
    double trackLen = 200.0;

    // setup simulation
    setupTest(test_info_->name(), 2);

    // set pre
    this->pre = [this, trackLen] (double simTime, double deltaTime) {

        // set lateral offset to track
        if(vehState->position.y > trackLen)
            _input.vehicle.d = 300.0; // just to set to a stupid value

        // set horizon points
        for(unsigned int i = 0; i < agent_model::NOH; ++i) {

            // subtract ego position
            double x = -vehState->position.x;
            double y = std::pow(0.5 * i, 2.0);

            // distance to end
            double sEnd = trackLen - vehState->position.y;

            // calculate horizon
            calculateHorizon(y, x, y, 0.0, i, sEnd);

        }

    };

    // set vehicle position
    vehState->position.x = 0.0;
    vehState->position.y = 0.0;
    vehState->psi = M_PI_2;

    // run simulation
    run();

    EXPECT_NEAR(0.0, vehState->position.x, 1e-3);
    EXPECT_NEAR(M_PI_2, vehState->psi, 1e-3);

}


/**
 * 2. Test: Horizon Control - Ending straight track
 *
 * Scenario:
 *  Given is a vehicle driver unit with a velocity of <init_speed>
 *  And a straight track with a length of <track_len> (x=0, y=0, phi=0)
 *  When the x-position of the vehicle is below <track_len>
 *  Then the y-position of the vehicle shall be zero (1e-3)
 *  When the simulation time of <sim_time> has passed
 *  Then the y-offset of the vehicle shall be near zero (0.1)
 *  And the vehicle shall move in x-direction (0.1)
 *
 * Values:
 *  sim_time = 30 s
 *  init_speed = 10 m/s
 *  track_len = 200 m
 */
TEST_F(AMT_HorizonControl, StraightEnding) {

    // setup simulation
    setupTest(test_info_->name(), 2);
    double trackLen = 200.0;

    // set pre
    this->pre = [this, trackLen] (double simTime, double deltaTime) {

        // set lateral offset to track
        if(vehState->position.x > trackLen)
            _input.vehicle.d = 300.0; // just to set to a stupid value

        // set horizon points
        for(unsigned int i = 0; i < agent_model::NOH; ++i) {

            // subtract ego position
            double x = std::pow(0.5 * i, 2.0);
            double y = -vehState->position.y;

            // distance to end
            double sEnd = trackLen - vehState->position.x - x;

            // calculate horizon
            calculateHorizon(x, x, y, 0.0, i, sEnd);

        }

    };


    this->post = [this, trackLen] (double simTime, double deltaTime) {

        if(vehState->position.x < trackLen) {

            EXPECT_NEAR(0.0, vehState->position.y, 1e-3);
            EXPECT_NEAR(0.0, vehState->psi, 1e-3);

        }

    };

    // run simulation
    run();

    EXPECT_NEAR(0.0, vehState->position.y, 0.1);
    EXPECT_NEAR(0.0, vehState->psi, 0.1);

}


/**
 * 3. Test: Horizon Control - Control defined offsets
 *
 * Scenario:
 *  Given is a vehicle driver unit with a velocity of <init_speed>
 *   And a straight track (x=0, y=0, phi=0)
 *   And at t = <t1> the desired offset is set to <off1>
 *   And at t = <t2> the desired offset is set to <off2> with a target distance of <dist2>
 *   And at t = <t3> the desired offset is set to zero
 *  When the simulation time is within the time interval <time_int1>
 *   Then the y-offset of the vehicle shall be zero (1e-2)
 *   And the vehicle shall move in x-direction (1e-2)
 *  When the simulation time is within the time interval <time_int2>
 *   Then the y-offset of the vehicle shall be <off1>
 *   And the vehicle shall move in x-direction (1e-2)
 *  When the simulation time is within the time interval <time_int3>
 *   Then the y-offset of the vehicle shall be <off2>
 *   And the vehicle shall move in x-direction (1e-3)
 *  When the simulation time of <sim_time> has passed
 *  Then the y-offset of the vehicle shall be zero (1e-3)
 *   And the vehicle shall move in x-direction (1e-3)
 *
 * Values:
 *  sim_time = 50 s
 *  init_speed = 10 m/s
 *  t1 = 10 s
 *  off1 = 1 m (+- 0.01)
 *  t2 = 20 s
 *  off2 = 1 m (+- 0.1)
 *  t3 = 40 s
 *  time_int1 = (0..10) s
 *  time_int2 = (15..20) s
 *  time_int3 = (30..40) s
 *
 */
TEST_F(AMT_HorizonControl, Offsets) {

    // set end time
    endTime = 50.0;

    // setup simulation
    setupTest(test_info_->name(), 3);

    // set pre
    this->pre = [this] (double simTime, double deltaTime) {

        // set offset decision
        if(simTime > 9.91 && simTime < 10.09) {

            // to the left side immediately
            decisions->lateral.distance = 0.0;
            decisions->lateral.value = 1.0;

        } else if(simTime > 19.91 && simTime < 20.09) {

            // to the right side over distance
            decisions->lateral.distance = 100.0;
            decisions->lateral.value = -1.0;

        } else if(simTime > 39.91 && simTime < 40.09) {

            // reset
            decisions->lateral.distance = 0.0;
            decisions->lateral.value = 0.0;

        }

        // set horizon points
        for(unsigned int i = 0; i < agent_model::NOH; ++i) {

            double s = std::pow(0.5 * i, 2.0);

            // subtract ego position
            double x = s;
            double y = -vehState->position.y;

            // calculate horizon
            calculateHorizon(s, x, y, 0.0, i);

        }

    };

    this->post = [this] (double simTime, double deltaTime) {

        if(simTime > 0.0 && simTime < 10.0) {
            EXPECT_NEAR(0.0, vehState->position.y, 1e-2);
            EXPECT_NEAR(0.0, vehState->psi, 1e-1);
        } else if(simTime > 15.0 && simTime < 20.0) {
            EXPECT_NEAR(1.0, vehState->position.y, 1e-2);
            EXPECT_NEAR(0.0, vehState->psi, 1e-1);
        } else if(simTime > 30.0 && simTime < 40.0) {
            EXPECT_NEAR(-1.0, vehState->position.y, 1e-1);
            EXPECT_NEAR(0.0, vehState->psi, 1e-1);
        }

    };

    // run simulation
    run();

    EXPECT_NEAR(0.0, vehState->position.y, 1e-3);
    EXPECT_NEAR(0.0, vehState->psi, 1e-3);

}


/**
 * 4. Test: Horizon Control - Circle
 *
 * Scenario:
 *  Given is a vehicle driver unit with a velocity of <init_speed>
 *  And the track is defined by a circle with a radius of <radius>
 *  When the simulation time is within the time interval <time_int>
 *  Then the vehicle shall have a distance to the origin of <radius>
 *  And the vehicle shall have a curvature of 1/<radius>
 *  And the vehicle shall have a yaw angle according to the tangent of the circle
 *
 * Values:
 *  sim_time = 30 s
 *  init_speed = 10.0 (+- 1 m/s)
 *  radius = 500.0 m (+- 0.1 m)
 */
TEST_F(AMT_HorizonControl, Circle) {

    endTime = 400.0;

    // setup simulation
    setupTest(test_info_->name(), 4);

    // set pre
    this->pre = [this] (double simTime, double deltaTime) {

        // set parameter and get current track position (as an angle)
        double phi = atan2(vehState->position.y, vehState->position.x);

        // update input (lateral offset and relative heading)
        _input.vehicle.d = sqrt(vehState->position.x * vehState->position.x
                + vehState->position.y * vehState->position.y) - radius;
        _input.vehicle.psi -= phi + M_PI_2;

        // set horizon points
        for(unsigned int i = 0; i < agent_model::NOH; ++i) {

            double s0 = std::pow(0.5 * i, 2.0);

            // get ego target position
            double psi = phi + s0 / radius;
            double x = radius * cos(psi) - vehState->position.x;
            double y = radius * sin(psi) - vehState->position.y;

            // calculate horizon
            calculateHorizon(s0, x, y, phi + M_PI_2, i);

        }

    };

    this->post = [this] (double simTime, double deltaTime) {

        // check radius
        if(simTime > 10.0) {

            // get angle (current track position)
            double phi = atan2(vehState->position.y, vehState->position.x);

            // check
            EXPECT_NEAR(radius, sqrt(vehState->position.x * vehState->position.x + vehState->position.y * vehState->position.y), 0.1);
            EXPECT_NEAR(1.0 / radius, vehState->kappa, 0.001);
            EXPECT_NEAR(0.0, std::fmod(phi + M_PI_2 - vehState->psi, 2.0 * M_PI), 0.01);

        }

    };

    // set vehicle position
    vehState->position.x =    0.0;
    vehState->position.y = -radius;

    // run simulation
    run();

}


/**
 * 5. Test: Horizon Control - Change path within time
 *
 * Scenario:
 *  TODO
 *
 * Values:
 *  TODO
 *
 */
TEST_F(AMT_HorizonControl, ChangePathWinthinTimeInterval) {

    // setup test
    setupTest(test_info_->name(), 1);

    this->pre = [this](double simTime, double timeStep) {

        // set lateral offset to track
        drInput->vehicle.d = vehState->position.y + drParam->vehicle.pos.y;
        drInput->vehicle.s = vehState->position.x + drParam->vehicle.pos.x;

        // set horizon points
        for(unsigned int i = 0; i < agent_model::NOH; ++i) {

            // subtract ego position
            double x  = std::pow(0.5 * i, 2.0);
            double y  = -vehState->position.y;

            // calculate horizon
            calculateHorizon(x, x, y, 0.0, i, INFINITY);

        }

        if(simTime > 9.99909 && simTime < 17.0001) {

            // calculate scale factor
            double f = agent_model::scale(simTime, 17.0, 10.0, 1.0);

            // set actual path
            conscious->lateral.paths[0].factor = (1.0 - f);
            conscious->lateral.paths[2].factor = f;

        } else if(simTime > 17.0) {

            conscious->lateral.paths[0].factor = 0.0;
            conscious->lateral.paths[2].factor = 1.0;

        } else if(simTime < 10.0) {

            conscious->lateral.paths[0].factor = 1.0;
            conscious->lateral.paths[2].factor = 0.0;

        }

    };

    this->post = [this](double simTime, double timeStep) {

        if(simTime <= 10.0)
            EXPECT_NEAR(0.0, vehState->position.y, 1e-3);
        else if(simTime >= 19.0)
            EXPECT_NEAR(3.75, vehState->position.y, 0.1);

    };

    // run test
    run();

}


/**
 * 6. Test: Horizon Control - Lane change
 *
 * Scenario:
 *  TODO
 *
 * Values:
 *  TODO
 *
 */
TEST_F(AMT_HorizonControl, PerformingLaneChange) {

    // lane ID
    int lane = 0;

    // setup test
    setupTest(test_info_->name(), 1);

    this->pre = [this, &lane](double simTime, double timeStep) {

        // set lateral offset to track
        drInput->vehicle.d = vehState->position.y + drParam->vehicle.pos.y - lane *  3.75;
        drInput->vehicle.s = vehState->position.x + drParam->vehicle.pos.x;

        // set horizon points
        for(unsigned int i = 0; i < agent_model::NOH; ++i) {

            // subtract ego position
            double x  = std::pow(0.5 * i, 2.0);
            double y  = -vehState->position.y + lane * 3.75;

            // calculate horizon
            calculateHorizon(x, x, y, 0.0, i, INFINITY);

        }

        // at 10 s, decide lane change
        if(simTime > 9.99909 && simTime < 10.0001) {

            // change lane
            decisions->laneChange = -1;

        }

    };

    this->post = [this, &lane](double simTime, double timeStep) {

        // change lane
        lane += drMemory->laneChange.switchLane;
        drMemory->laneChange.switchLane = 0;

    };

    // run test
    run();

}



/**
 * 6. Test: Horizon Control - Lane change
 *
 * Scenario:
 *  TODO
 *
 * Values:
 *  TODO
 *
 */
TEST_F(AMT_HorizonControl, DecidingLaneChange) {

    // lane ID
    int lane = 0;
    endTime = 60.0;

    // setup test
    setupTest(test_info_->name(), 1);

    this->pre = [this, &lane](double simTime, double timeStep) {

        // set lateral offset to track
        drInput->vehicle.d = vehState->position.y + drParam->vehicle.pos.y - lane *  3.75;
        drInput->vehicle.s = vehState->position.x + drParam->vehicle.pos.x;

        // set horizon points
        for(unsigned int i = 0; i < agent_model::NOH; ++i) {

            // subtract ego position
            double x  = std::pow(0.5 * i, 2.0);
            double y  = -vehState->position.y + lane * 3.75;

            // calculate horizon
            calculateHorizon(x, x, y, 0.0, i, INFINITY);

        }

        // set target
        drInput->targets[0].id = 1;
        drInput->targets[0].ds = 400.0 - drInput->vehicle.s;
        drInput->targets[0].d = 0.0;
        drInput->targets[0].a = 0.0;
        drInput->targets[0].v = 0.0;
        drInput->targets[0].psi = 0.0;
        drInput->targets[0].lane = -lane;
        drInput->targets[0].size.length = 5.0;
        drInput->targets[0].size.width  = 2.2;
        drInput->targets[0].xy.x = 400.0 - drInput->vehicle.s;
        drInput->targets[0].xy.y = -drInput->vehicle.d;

        // set target
        drInput->targets[1].id = 2;
        drInput->targets[1].ds = 400.0 - drInput->vehicle.s;
        drInput->targets[1].d = 0.0;
        drInput->targets[1].a = 0.0;
        drInput->targets[1].v = 0.0;
        drInput->targets[1].psi = 0.0;
        drInput->targets[1].lane = 1 - lane;
        drInput->targets[1].size.length = 5.0;
        drInput->targets[1].size.width  = 2.2;
        drInput->targets[1].xy.x = 400.0 - drInput->vehicle.s;
        drInput->targets[1].xy.y = -drInput->vehicle.d + 3.75;



    };

    this->post = [this, &lane](double simTime, double timeStep) {

        // change lane
        lane += drMemory->laneChange.switchLane;
        drMemory->laneChange.switchLane = 0;

    };

    // run test
    run();

}