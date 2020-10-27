// Copyright (c) 2020 Institute for Automotive Engineering (ika), RWTH Aachen University. All rights reserved.
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
// Created by Jens Klimke on 2020-07-10.
// Contributors:
//
// AMT_Longitudinal.cpp
//


#include <simcore/functions.h>
#include <simcore/value/SignalTube.h>
#include <fstream>
#include "AgentModelAdapter.h"


struct AMT_Longitudinal : public AgentModelAdapter {

    double initialVelocity = 0.0;
    double initialPedal = 0.0;

    double xTarget = 700.0;
    double vTarget = 30.0;

    double xSpeedLimit1 = 700.0;
    int vSpeedLimit0 = 80.0;
    int vSpeedLimit1 = 50.0;

    double dsNet = INFINITY;

    agent_model::Target *target0 = nullptr;

    std::fstream fs;

    AMT_Longitudinal() = default;
    ~AMT_Longitudinal() override = default;


    void setupTest(std::string &&label, unsigned int no) {

        // setup end conditions
        setup.endTime = 160.0;
        setup.endDistance = INFINITY;

        // create simulation
//        setup.logFile = sim::fnc::string_format("AMT_AccelerationControl_%s.json", label.c_str());
        setup.logTitle = sim::fnc::string_format("Test 10.%d", no);

        // create simulation
        create();

        // initialize target
        target0 = &drInput->targets[0];
        initTarget(target0, 1, xTarget, vTarget / 3.6);

        // set initial speed
        vehState->v = initialVelocity;
        vehInput->pedal = initialPedal;

        if(EN_LOGGING) {

            // add target speed
            jsonLogger.addValue("vTar", &target0->v);
            plotLogger.trace("velocity", "vTar", "time", "vTar", "auto", 2);

            // add distance to log
            jsonLogger.addValue("ds", &dsNet);
            plotLogger.addFigure("distance", "Distance to following target", "time t [s]", "distance ds [m]", "time",
                    "ds", false);

        }

    }

    void preStep(double simTime, double deltaTime) override {

        // set speed limits
        drInput->signals[0].id = 1;
        drInput->signals[0].ds = -1.0 - vehState->s;
        drInput->signals[0].type = agent_model::SIGNAL_SPEED_LIMIT;
        drInput->signals[0].value = vSpeedLimit0;
        drInput->signals[1].id = 2;
        drInput->signals[1].ds = xSpeedLimit1 - vehState->s;
        drInput->signals[1].type = agent_model::SIGNAL_SPEED_LIMIT;
        drInput->signals[1].value = vSpeedLimit1;

        // update target and calculate net distance (for checking and logging)
        updateTarget(target0, xTarget, deltaTime);
        dsNet = (xTarget - target0->size.length * 0.5) - (vehState->position.x + vehParam->size.x * 0.5);

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


TEST_F(AMT_Longitudinal, PublicationTest1) {

    // setup test
    setupTest(test_info_->name(), 1);

    fs.open((std::string(LOG_DIR) + "/data/" +  test_info_->name() + "_01.csv").c_str(), std::ios::out);
    fs << "t,v,a,s,ds" << std::endl;

    // set pre
    this->pre = [this] (double, double) {

        _param.follow.timeHeadway = 1.0;
        _param.follow.thwMax = 10.0;

    };

    // set post
    this->post = [this] (double simTime, double) {

        // check speed limit reached
        if(simTime >= 22.0 && simTime < 25.0)
            EXPECT_NEAR(80.0, vehState->v * 3.6, 1.6);

        // check speed step reached
        if(vehState->s >= 700 && vehState->s < 720)
            EXPECT_NEAR(50.0, vehState->v * 3.6, 1.0);

        // start reacting on speed limit
        if(vehState->s < 477.77)
            EXPECT_GE(vehState->a, -1e-4);
        else if(vehState->s > 480.0 && vehState->s < 500.0)
            EXPECT_LE(vehState->a, -1e-3);

        fs << simTime << "," << vehState->v << "," << vehState->a << "," << vehState->s << "," << drState->conscious.follow.distance << std::endl;

    };

    // run simulation
    run();

    // close file
    fs.close();

    // check final distance
    EXPECT_NEAR(30.0 / 3.6, dsNet, 1e-3);

}
