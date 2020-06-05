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
// Created by Jens Klimke on 2019-03-23.
// Contributors:
//
// AgentModel.h

#ifndef AGENT_MODEL_H
#define AGENT_MODEL_H

#include "Interface.h"
#include "VelocityHorizon.h"
#include "StopHorizon.h"
#include "Filter.h"
#include "DistanceTimeInterval.h"


/**
 * @brief The agent model main class
 */
class AgentModel : public agent_model::Interface {


protected:

    agent_model::StopHorizon _stop_horizon{};                         //!< attribute to store the stop points
    agent_model::VelocityHorizon _vel_horizon{};                      //!< attribute to store the stop points
    agent_model::Filter _filter{};                                    //!< attribute to store the speed reaction filter
    agent_model::DistanceTimeInterval _lateral_offset_interval;       //!< attribute to store the lateral offset interval
    agent_model::DistanceTimeInterval _lane_change_process_interval;  //!< attribute to store the lane change interval


public:


    /**
     * Default constructor
     */
    AgentModel() = default;


    /**
     * Default destructor
     */
    ~AgentModel() override = default;


    /**
     * Initializes the driver model. Shall be ran before the the first step is executed.
     */
    void init();


    /**
     * Performs a driver model step.
     * The driver model must be initializes (@see init()).
     * @param simulationTime The current simulation time
     */
    void step(double simulationTime);


protected:

    /**
     * Calculates process of stopping and starting
     */
    void decisionProcessStop();


    /**
     * Calculates the decision to perform a lane change
     */
    void decisionLaneChange();


    /**
     * Calculates the decision to perform a lateral offset
     */
    void decisionLateralOffset();


    /**
     * Calculates the target speed based on rules, the curvature of the track
     */
    void consciousVelocity();


    /**
     * Calculate the process of the stop maneuver
     */
    void consciousStop();


    /**
     * Calculates the net distance to the relevant following traffic participants
     */
    void consciousFollow();

    /**
     * Calculates the lane change process
     */
    void consciousLaneChange();


    /**
     * Calculates the offset to be added to the reference points
     */
    void consciousLateralOffset();


    /**
     * Calculates the reference points for the lateral motion control
     */
    void consciousReferencePoints();


    /**
     * Calculates the reaction for the lateral motion control based on the reference points
     * @return The reaction value for lateral motion control
     */
    double subconsciousLateralControl();


    /**
     * Calculates the reaction to follow other traffic participants
     * @return The reaction value to follow
     */
    double subconsciousFollow();


    /**
     * Calculates the reaction to stop the vehicle at the desired point
     * @return The reaction value to stop
     */
    double subconsciousStop();


    /**
     * Calculates the reaction to reach the desired speed, including predictive control
     * @return The reaction value to control speed
     */
    double subconsciousSpeed();


    /**
     * Calculates the pedal behavior when starting or stopping for sub-microscopic simulations
     * @return The pedal value
     */
    double subconsciousStartStop();


};


#endif // AGENT_MODEL_H
