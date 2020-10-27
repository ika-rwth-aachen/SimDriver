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
// Created by Jens Klimke on 2019-11-12.
//

#include <string>
#include <functional>
#include <gtest/gtest.h>
#include <simdriver/Behavior.h>
#include <simcore/ISynchronized.h>
#include <simcore/utils/functions.h>
#include <simtraffic/VehicleModelAdapter.h>
#include "VehicleControllerAdapter.h"

#ifndef SIM_DRIVER_AGENT_MODEL_ADAPTER_H
#define SIM_DRIVER_AGENT_MODEL_ADAPTER_H


/**
 * A abstract generic test class, setting a default set of parameters for the vehicle and driver model
 * @author Jens Klimke
 */
struct AgentModelAdapter : public ::testing::Test, public sim::traffic::VehicleModelAdapter {

    struct Scenario {

        // sim setup
        double stepSize = 0.1;
        double endTime = INFINITY;
        double endDistance = INFINITY;

        // logging
        std::string logTitle{};

    } setup{};

    agent_model::__Conscious *conscious = nullptr;
    agent_model::__Subconscious *subconscious = nullptr;
    agent_model::__Decisions *decisions = nullptr;

    std::function<void (double, double)> pre{};
    std::function<void (double, double)> post{};

    PlotLogger plotLogger{};
    JsonFileReporter jsonLogger{};

    VehicleControllerAdapter adapter{};

    AgentModelAdapter() = default;
    ~AgentModelAdapter() override = default;

    void TearDown() override {
        destroy();
    }


    /**
     * @brief Creates a simulation.
     *
     * In this method a new simulation is created with all its new elements (like agents, reporters, loop, etc.) After
     * that the vehicle and driver model parameters are set. Finally the state of the vehicle is reset and the agent
     * models are placed to the initial position.
     *
     */
    void create() override {

        // set time step size for vehicle and controllers
        adapter.setTimeStepSize(0.01);

        // link vehicle and controllers
        vehicle = &adapter.vehicle;
        pedalContr = &adapter.pedal;
        steeringContr = &adapter.steer;

        // link vehicle
        vehInput = vehicle->getInput();
        vehState = vehicle->getState();
        vehParam = vehicle->getParameters();

        // link driver
        drInput = getInput();
        drState = getState();
        drParam = getParameters();
        drMemory = getMemory();

        // destroy and create simulation
        BasicSimulation::destroy();
        BasicSimulation::create(setup.endTime, 0.001, false, {{&vehState->s, setup.endDistance}});

        // set time step size
        Model::setTimeStepSize(setup.stepSize);

        // add as components
        addComponent(&adapter);
        addComponent(this);

        // set logging files
        auto plotFile = sim::fnc::string_format("%s/plot.json", LOG_DIR);
        auto dataFile = sim::fnc::string_format("%s/data.json", LOG_DIR);

        // logging
        if(EN_LOGGING) {

            // setup loggers
            plotLogger.create(plotFile, setup.logTitle);
            plotLogger.setDataFile(dataFile);
            jsonLogger.setFilename(dataFile);

            // set time step size
            plotLogger.setTimeStepSize(setup.stepSize);
            jsonLogger.setTimeStepSize(setup.stepSize);

            // setup plot data
            setupPlotData();

            // add to simulation
            addComponent(&plotLogger);
            addComponent(&jsonLogger);

        }

        // set parameters
        double l = 2.7; // wheel base

        // vehicle parameters
        vehParam->steerTransmission  = 0.474;
        vehParam->wheelBase          = l;
        vehParam->cwA                = 0.6;
        vehParam->mass               = 1.5e3;
        vehParam->powerMax           = 1.0e5;
        vehParam->forceMax           = 1.0e4;
        vehParam->idle               = 0.05;
        vehParam->rollCoefficient[0] = 4.0 * 9.91e-3;
        vehParam->rollCoefficient[1] = 4.0 * 1.95e-5;
        vehParam->rollCoefficient[2] = 4.0 * 1.76e-9;
        vehParam->size.x             = 5.0;
        vehParam->size.y             = 2.2;
        vehParam->driverPosition.x   = 0.5;
        vehParam->driverPosition.y   = 0.5;

        // sync vehicle driver parameters
        drParam->vehicle.pos.x = vehParam->driverPosition.x;
        drParam->vehicle.pos.y = vehParam->driverPosition.y;
        drParam->vehicle.size.length = vehParam->size.x;
        drParam->vehicle.size.width  = vehParam->size.y;

        // lane change
        drParam->laneChange.time = 7.0;
        drParam->laneChange.bSafe = 0.5;
        drParam->laneChange.aThreshold = 0.5;
        drParam->laneChange.politenessFactor = 0.5;

        // speed control
        drParam->velocity.a =  2.0;
        drParam->velocity.b = -2.0;
        drParam->velocity.thwMax = 10.0;
        drParam->velocity.delta = 4.0;
        drParam->velocity.deltaPred = 1.0;
        drParam->velocity.vComfort = 120.0 / 3.6;
        drParam->velocity.ayMax = 1.3;

        // stop control
        drParam->stop.T = 2.0;
        drParam->stop.TMax = 10.0;
        drParam->stop.dsMax = 20.0;
        drParam->stop.tSign = 1.0;
        drParam->stop.dsGap = 2.0;
        drParam->stop.vStopped = 0.5;
        drParam->stop.pedalDuringStanding = -0.3;

        // following
        drParam->follow.dsStopped = 2.0;
        drParam->follow.thwMax = 10.0;
        drParam->follow.timeHeadway = 1.5;

        // steering
        drParam->steering.thw[0] = 1.0;
        drParam->steering.thw[1] = 2.0;
        drParam->steering.dsMin[0] =  5.0;
        drParam->steering.dsMin[1] = 10.0;
        drParam->steering.P[0] = 0.06 * l;
        drParam->steering.P[1] = 0.03 * l;
        drParam->steering.D[0] = 0.0;
        drParam->steering.D[1] = 0.0;

        // set controller parameters (lateral motion control)
        steeringContr->setParameters(10.0 * l, 0.1 * l, 0.0, 1.0);
        steeringContr->setRange(-1.0, 1.0, INFINITY);

        // set controller parameters (longitudinal motion control)
        pedalContr->setParameters(2.5, 1.0, 0.0, 1.0);
        pedalContr->setRange(-1.0, 1.0, INFINITY);

        // set states
        vehicle->reset();

        // set variables
        pedalContr->setVariables(&vehState->a, &drState->subconscious.a, &vehInput->pedal, &drState->subconscious.pedal);
        steeringContr->setVariables(&vehState->kappa, &drState->subconscious.kappa, &vehInput->steer);

        // create injection structure
        agent_model::registerTree(&injInput,         drInput,                drInput);
        agent_model::registerTree(&injParameters,    drParam,                drParam);
        agent_model::registerTree(&injMemory,        drMemory,               drMemory);
        agent_model::registerTree(&injDecisions,    &drState->decisions,    &drState->decisions);
        agent_model::registerTree(&injConscious,    &drState->conscious,    &drState->conscious);
        agent_model::registerTree(&injSubconscious, &drState->subconscious, &drState->subconscious);

        // set shortcut
        conscious = &injConscious;
        subconscious = &injSubconscious;
        decisions = &injDecisions;

    }


    void setupPlotData() {

        typedef std::vector<std::pair<std::string, std::string>> p;

        // relevant velocities
        p velocityPairs =
                {{"velocity",       "v"},
                 {"local vel.",     "vLoc"},
                 {"predicted vel.", "vPred"}};

        // relevant accelerations
        p accelerationPairs =
                {{"acceleration", "a"},
                 {"desired acc.", "aDes"}};

        // add data
        plotLogger.addFigure("position", "Position", "x [m]", "y [m]", "x", "y", true);
        plotLogger.addFigure("velocity", "Velocity", "time [s]", "velocity [m/s]", "time", velocityPairs);
        plotLogger.addFigure("acceleration", "Acceleration", "time [s]", "acceleration [m/s^2]", "time", accelerationPairs);
        plotLogger.addFigure("pedal", "Pedal value", "time [s]", "pedal [-]", "time", "pedal");

        // logging data
        jsonLogger.addValue("x",         &vehState->position.x);
        jsonLogger.addValue("y",         &vehState->position.y);
        jsonLogger.addValue("a",         &vehState->a);
        jsonLogger.addValue("ay",        &vehState->ay);
        jsonLogger.addValue("aDes",      &drState->subconscious.a);
        jsonLogger.addValue("v",         &vehState->v);
        jsonLogger.addValue("s",         &vehState->s);
        jsonLogger.addValue("psi",       &vehState->psi);
        jsonLogger.addValue("dPsi",      &vehState->dPsi);
        jsonLogger.addValue("kappa",     &vehState->kappa);
        jsonLogger.addValue("kappaDes",  &drState->subconscious.kappa);
        jsonLogger.addValue("delta",     &vehState->delta);
        jsonLogger.addValue("pedal",     &vehInput->pedal);
        jsonLogger.addValue("steer",     &vehInput->steer);
        jsonLogger.addValue("theta0",    &drState->aux[0]);
        jsonLogger.addValue("theta1",    &drState->aux[6]);
        jsonLogger.addValue("dtheta0",   &drState->aux[1]);
        jsonLogger.addValue("dtheta1",   &drState->aux[7]);
        jsonLogger.addValue("dy",        &drInput->vehicle.d);
        jsonLogger.addValue("ds",        &drState->conscious.follow.distance);
        jsonLogger.addValue("vLoc",      &drState->conscious.velocity.local);
        jsonLogger.addValue("vPred",     &drState->conscious.velocity.prediction);
        jsonLogger.addValue("dy0",       &drState->conscious.lateral.paths[0].refPoints[0].y);
        jsonLogger.addValue("dy1",       &drState->conscious.lateral.paths[0].refPoints[1].y);

    }


    /**
     * Destroys the unit
     */
    void destroy() override {

        // remove injection structure
        InjectionInterface::remove(drInput);
        InjectionInterface::remove(drParam);
        InjectionInterface::remove(drMemory);
        InjectionInterface::remove(&drState->decisions);
        InjectionInterface::remove(&drState->conscious);
        InjectionInterface::remove(&drState->subconscious);

    }


    /**
     * Initializes the model
     * @param initTime Simulation time at initialization
    */
    void initialize(double initTime) override {

        // initialize model
        sim::ISynchronized::initialize(initTime);

        // initialize agent model
        AgentModel::init();

    }


    /**
     * @brief Executes code before the actual agent model step is executed.
     * Also executes the pre function pointer if set.
     * @param simTime Simulation time
     * @param deltaTime Time step size from the last step
     */
    virtual void preStep(double simTime, double deltaTime) {

        // run pre step method
        if(pre)
            pre(simTime, deltaTime);

    }


    /**
     * @brief Executes code after the actual agent model step is executed.
     * Also executes the post function pointer if set.
     * @param simTime Simulation time
     * @param deltaTime Time step size from the last step
     */
    virtual void postStep(double simTime, double deltaTime) {

        // run pre step method
        if(post)
            post(simTime, deltaTime);

    }


    /**
     * @brief Method to be called for each step.
     *
     * The method writes the driver model input and runs the pre step method. Then the step method of the driver model
     * is executed. After that the position of the agent is updated (including map matching), the post method is
     * executed.
     *
     * @param simTime The actual simulation time
     * @return A flag denoting whether the simulation step is executed or not
     */
    bool step(double simTime) override {

        if(!Model::step(simTime))
            return false;

        // update ego position
        updatePosition();

        // write inputs
        writeInput();

        // get time step size
        double deltaTime = timeStep(simTime);

        // run pre step method
        preStep(simTime, deltaTime);

        // run driver model step
        AgentModel::step(simTime);

        // run post step method
        postStep(simTime, deltaTime);

        // ok!
        return true;

    }

    /**
     * Does nothing in this implementation
     */
    void updatePosition() override {}


    /**
     * Writes the input for the driver model
     */
    void writeInput() override {

        // copy vehicle model state
        drInput->vehicle.a        = vehState->a;
        drInput->vehicle.v        = vehState->v;
        drInput->vehicle.dPsi     = vehState->dPsi;
        drInput->vehicle.s        = vehState->s;
        drInput->vehicle.pedal    = vehInput->pedal;
        drInput->vehicle.steering = vehInput->steer;

        // copy relative state (expected is, that the track is straight in x direction with y = 0)
        drInput->vehicle.d   = vehState->position.y;
        drInput->vehicle.psi = vehState->psi;

        // set all horizon points to inf
        for(double & ds : drInput->horizon.ds)
            ds = INFINITY;

        // set all signals to inf
        for(auto & signal : drInput->signals)
            signal.ds = INFINITY;

        for(auto & target : drInput->targets)
            target.ds = INFINITY;

    }

};


#endif // SIM_DRIVER_AGENT_MODEL_ADAPTER_H
