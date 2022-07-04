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
// Created by Jens Klimke on 2019-03-23.
// Contributors:
//
// AgentModel.cpp

#include <cmath>
#include <algorithm>
#include <iostream>
#include <limits>

#include "AgentModel.h"
#include "model_collection.h"

#ifndef NEG_INFINITY
#define NEG_INFINITY (-1.0 * INFINITY)
#endif

#if WITH_INJECTION
#include <injection/Injection.h>
#define APPLY(PNTR) { InjectionInterface::applyAll(PNTR); InjectionInterface::resetAll(PNTR); }
#else
#define APPLY(PNTR)
#endif

static const int LOW_SPEED = 5.0;
static const double AM_CLOSE_TO_ONE = 0.999999999;


void AgentModel::init() {

    // unset distance counter
    _memory.vehicle.s = 0.0;

    // unset lane change
    _memory.laneChange.switchLane = 0;

    // unset lateral control memory
    _memory.lateral.startDistance   = INFINITY;
    _memory.lateral.startTime       = INFINITY;

    // unset velocity
    _memory.velocity = INFINITY;

    // set lane change
    _memory.laneChange.startTime = INFINITY;

    // init horizon
    _stop_horizon.init(_input.vehicle.s);
    _vel_horizon.init(_input.vehicle.s, 401);
    _filter.init(10);

    // init lateral control
    _lateral_offset_interval.reset();
    _lateral_offset_interval.setScale(0.0);

    // init lane change process
    _lane_change_process_interval.reset();
    _lane_change_process_interval.setDelta(0.3);
    _lane_change_process_interval.setScale(0.0);

    // init road priorities
    _state.conscious.stop.priority = false;
    _state.conscious.stop.give_way = false;

}


void AgentModel::step(double simulationTime) {

    // TODO: Think about using fixed horizon points. This would avoid changing curvature.

    // apply injection for parameters and inputs
    APPLY(&this->_param)
    APPLY(&this->_input)
    APPLY(&this->_memory)

    // update internal horizons
    _stop_horizon.update(_input.vehicle.s, simulationTime);
    _vel_horizon.update(_input.vehicle.s);
    _lateral_offset_interval.update(_input.vehicle.s, simulationTime);
    _lane_change_process_interval.update(_input.vehicle.s, simulationTime);

    // set time
    _state.simulationTime = simulationTime;

    // decisions
    decisionLaneChange();    // open
    decisionProcessStop();   // done: Test 10.1, 10.2
    decisionLateralOffset(); // done: no implementation yet

    // apply injection for decision
    APPLY(&this->_state.decisions)

    // conscious calculation
    consciousVelocity();                // done: Test 9.1, 9.2, 9.3
    consciousStop();                    // done: Test 3.3b, 3.3c
    consciousFollow();                  // done: Test 8.1, 8.2, 8.3
    consciousLaneChange();              // open
    consciousLateralOffset();           // done: Test 7.3
    consciousReferencePoints();         // done: Test 7.1, 7.2, 7.3, 7.4

    // apply injection for conscious states
    APPLY(&this->_state.conscious)

    // calculate speed reaction
    double rSpeed  = subconsciousSpeed();          // done: Test 2.1, 2.2, 2.3, 3.1, 3.2, 3.6
    double rStop   = subconsciousStop();           // done: Test 3.3, 3.6
    double rFollow = subconsciousFollow();         // done: Test 3.4, 3.5, 3.6
    double pedal   = subconsciousStartStop();      // done: Test 1.1, 1.2
    double kappa   = subconsciousLateralControl(); // done: Test 6.1, 6.2, 6.3, 6.4	
    // calculate resulting acceleration
    double aRes = _param.velocity.a * (1.0 - rSpeed - rStop - rFollow);

    // set desired values
    _state.subconscious.a     = std::min(std::max(-10.0, aRes), 10.0);           // done: Test 1.3
    _state.subconscious.kappa = kappa;          // done: Test 4.1, 4.2
    _state.subconscious.pedal = pedal;          // done: Test 1.4
    _state.subconscious.steering = INFINITY;

    // apply injection for sub-conscious states
    APPLY(&this->_state.subconscious)

    // save values to memory
    _memory.vehicle.s = _input.vehicle.s;

}


void AgentModel::decisionProcessStop() {

    // unset decision
    _state.decisions.signal.id = std::numeric_limits<unsigned int>::max();
    _state.decisions.signal.position = INFINITY;
    _state.decisions.signal.standingTime = INFINITY;

    _state.decisions.target.id = std::numeric_limits<unsigned int>::max();
    _state.decisions.target.position = INFINITY;
    _state.decisions.target.standingTime = INFINITY;

    _state.decisions.destination.id = std::numeric_limits<unsigned int>::max();
    _state.decisions.destination.position = INFINITY;
    _state.decisions.destination.standingTime = INFINITY;

    // add stop point because of destination point
    if (_input.horizon.destinationPoint > 0)
    {
        _state.decisions.destination.id = 3;
        _state.decisions.destination.position = _input.vehicle.s + _input.horizon.destinationPoint;
        _state.decisions.destination.standingTime = INFINITY;
    }

    // not yet decided about to stop or drive
    bool stop = false;
    bool drive = false;
    bool found_signal = false;

    // iterate over all signals and mark ds of the next relavant signal
    double ds_rel_tls = INFINITY;
    double ds_rel_sgn = INFINITY;
    agent_model::Signal* rel;
    agent_model::Signal* rel_tls;
    agent_model::Signal* rel_sgn;

    for(auto &e : _input.signals) 
    {
        if (e.type == agent_model::SignalType::SIGNAL_TLS && 
            e.ds >= 0 && e.ds < ds_rel_tls) 
        {
            ds_rel_tls = e.ds;
            rel_tls = &e;
            found_signal = true;
        }
        if ((e.type == agent_model::SignalType::SIGNAL_YIELD ||
            e.type == agent_model::SignalType::SIGNAL_PRIORITY ||
            e.type == agent_model::SignalType::SIGNAL_STOP) && 
            e.sign_is_in_use &&
            !e.subsignal &&
            e.ds >= 0 && e.ds < ds_rel_sgn) 
        {
            ds_rel_sgn = e.ds;
            rel_sgn = &e;
            found_signal = true;
        }   
    }

    // take closest signal (take traffic light even if 10 meters behind sign)
    if (ds_rel_tls <= ds_rel_sgn + 10) {
        rel = rel_tls;
    } else {
        rel = rel_sgn;
    }

    if(found_signal) 
    {   
        // calculate net distance
        auto ds = rel->ds - _param.stop.dsGap + _param.vehicle.pos.x - _param.vehicle.size.length * 0.5;

        // trafficlight
        if (rel->type == agent_model::SignalType::SIGNAL_TLS)
        {
            // case red trafficlight
            if (rel->color == agent_model::TrafficLightColor::COLOR_RED) {
                
                _state.conscious.stop.give_way = true;
                
                // set stop point for INFINITY ( = until removed)
                _state.decisions.signal.id = 1;
                _state.decisions.signal.position = _input.vehicle.s + ds;
                _state.decisions.signal.standingTime = INFINITY;
                
                stop = true;
                return;
            }

            // case green trafficlight
            else if (rel->color == agent_model::TrafficLightColor::COLOR_GREEN) {
                _state.conscious.stop.priority = true;

                // "remove" stop point (by setting standing time = 0)
                _state.decisions.signal.id = 1;
                _state.decisions.signal.position = _input.vehicle.s + ds;
                _state.decisions.signal.standingTime = 0;

                // drive if green and straight
                if (_input.vehicle.maneuver == agent_model::Maneuver::STRAIGHT)
                    drive = true;

                // drive if green-left-arrow and left turn
                if (rel->icon == agent_model::TrafficLightIcon::ICON_ARROW_LEFT 
                && _input.vehicle.maneuver == agent_model::Maneuver::TURN_LEFT)
                    drive = true;
            } 
        }
                
        // signal
        if (rel->type != agent_model::SignalType::SIGNAL_TLS)
        {
            // case stop signal
            if (rel->type == agent_model::SignalType::SIGNAL_STOP) {
                _state.conscious.stop.give_way = true;
                stop = true;
            }
            
            // case yield signal
            else if (rel->type == agent_model::SignalType::SIGNAL_YIELD) {
                _state.conscious.stop.give_way = true;
            }

            // case priority signal
            else if (rel->type == agent_model::SignalType::SIGNAL_PRIORITY) {
                _state.conscious.stop.priority = true;
            }
        }
    }

    // add stop point because of signal
    if (stop)
    {
        _state.decisions.signal.id = 1;
        _state.decisions.signal.position = _input.vehicle.s + rel->ds;
        _state.decisions.signal.standingTime = _param.stop.tSign;
    }

    // if not yet decided to drive or stop -> consider targets
    if (!drive && !stop) {   

        // ignore if not approaching intersection
        if (_input.vehicle.dsIntersection == INFINITY)
            return;
        
        // ignore if on priority lane and driving straight (or right - for now)
        if (_state.conscious.stop.priority && 
           (_input.vehicle.maneuver == agent_model::Maneuver::STRAIGHT ||
            _input.vehicle.maneuver == agent_model::Maneuver::TURN_RIGHT))
            return;

        // process all relevant targets
        for (auto &t : _input.targets)
        {
            // ignore unset targets
            if (t.id == 0) continue;

            // ignore targets not in junction area
            if (t.position == agent_model::TARGET_NOT_RELEVANT) 
                continue;
                
            // ignore targets on path
            if (t.position == agent_model::TARGET_ON_PATH) 
                continue;

            // ego is on priority
            if (_state.conscious.stop.priority)
            {   
                // normaly do not stop when on priority lane
                if (_input.vehicle.maneuver == agent_model::Maneuver::STRAIGHT && _input.vehicle.maneuver == agent_model::Maneuver::TURN_RIGHT)
                {
                    continue;
                }
                // special case for left turn
                else if (_input.vehicle.maneuver == agent_model::Maneuver::TURN_LEFT)
                {
                    if (t.position == agent_model::TARGET_ON_OPPOSITE)
                    {
                        stop = true;
                    }
                    continue;
                }
            }

            // ego is on give way lane
            if (_state.conscious.stop.give_way)
            {
                // stop for target already on intersection
                if (t.position == agent_model::TargetPosition::TARGET_ON_INTERSECTION)
                {
                    stop = true;
                    continue;
                }

                // stop if target prority not set (passive behavior)
                if (t.priority == agent_model::TargetPriority::TARGET_PRIORITY_NOT_SET) 
                {
                    stop = true;
                    continue;
                }

                // if target has to give way as well (first come, first drive)
                if (t.priority == agent_model::TargetPriority::TARGET_ON_GIVE_WAY_LANE) 
                {   
                    // special cases

                    // opposite target and left turn -> stop
                    if (t.position == agent_model::TARGET_ON_OPPOSITE && _input.vehicle.maneuver == agent_model::Maneuver::TURN_LEFT)
                    {
                        if (_input.vehicle.maneuver == agent_model::Maneuver::TURN_LEFT)
                        stop = true;
                        continue;
                    }
                       
                    // opposite target and not left turn -> continue
                    if (t.position == agent_model::TARGET_ON_OPPOSITE && _input.vehicle.maneuver != agent_model::Maneuver::TURN_LEFT)
                    {
                        continue;
                    }

                    // right target and right turn -> continue
                    if (t.position == agent_model::TARGET_ON_RIGHT && _input.vehicle.maneuver == agent_model::Maneuver::TURN_RIGHT)
                    {
                        continue;
                    }                 

                    // if no special case, check if ego reaches junction earlier
                    if (_input.vehicle.dsIntersection / _input.vehicle.v < t.dsIntersection / t.v) 
                    {
                        continue;
                    } 
                    // if target vehicle approaches intersection earlier
                    else 
                    {
                        stop = true;
                        continue;
                    }
                }

                // stop for target on priority lane
                if (t.priority == agent_model::TargetPriority::TARGET_ON_PRIORITY_LANE)
                {
                    stop = true;
                    continue;
                }
            }
            
            // ego is not on priority and give way lane (right before left)
            if (!_state.conscious.stop.priority && !_state.conscious.stop.give_way) 
            {
                if (t.position == agent_model::TARGET_ON_INTERSECTION)
                {
                    stop = true;
                    continue;
                }
                if (t.position == agent_model::TARGET_NOT_RELEVANT) 
                {
                    continue;
                }
                if (t.position == agent_model::TARGET_ON_OPPOSITE &&
                    _input.vehicle.maneuver == agent_model::Maneuver::TURN_LEFT)
                {
                    stop= true;
                    continue;
                }
                if (t.position == agent_model::TARGET_ON_RIGHT) 
                {
                    stop= true;
                    continue;
                }
                if (t.position == agent_model::TARGET_ON_LEFT)
                {
                    continue;
                }
            }
        }
        
        // add stop point because of target
        if (stop)
        {
            // try to stop 10m before intersection or take ds of the signal
            double ds_stop;
            if (std::isinf(ds_rel_sgn))
                ds_stop = std::max(0.0, _input.vehicle.dsIntersection - 10);
            else
                ds_stop = std::max(0.0, rel->ds);
            _state.decisions.target.id = 2;
            _state.decisions.target.position = _input.vehicle.s + ds_stop;
            _state.decisions.target.standingTime = _param.stop.tSign;
        }
    }
}

void AgentModel::decisionLaneChange() {

    // check for route-based lane changes

    // get current lanes
    agent_model::Lane* ego = nullptr;
    agent_model::Lane* left = nullptr;
    agent_model::Lane* right = nullptr;

    for (auto &lane : _input.lanes) {

        if (lane.id == 0) { 
            ego = &lane;
        }   
        if (lane.id == 1) { 
            left = &lane;
        }   
        if (lane.id == -1) { 
            right = &lane;
        } 
    }
    
    int lane_change;

    // if desired lane_change, right/left lane accessible, and route longer 
    if (ego && left && ego->lane_change && left->access == agent_model::ACC_ACCESSIBLE && left->route >= ego->route) {
        lane_change = 1;
    }
    else if (ego && right && ego->lane_change && right->access == agent_model::ACC_ACCESSIBLE && right->route >= ego->route) {
        lane_change = -1;
    }
    else {
        lane_change = 0;
    }

    // check if lane is occupied by a target
    if (lane_change != 0) {
        for (auto &target : _input.targets) {

            // get target
            auto tar = &target;

            if (tar->lane == lane_change && abs(tar->ds) <= 10) {
                lane_change = 0;
                break; 
            }
        }
    }

    // set final lane_change
    _state.decisions.laneChange = lane_change;
    
    return; // do not consider MOBIL model on intersections

    // check positions
    double dsLF = INFINITY;
    double vLF = 0.0;
    double dsLB = INFINITY;
    double vLB = 0.0;
    double dsRF = INFINITY;
    double vRF = 0.0;
    double dsRB = INFINITY;
    double vRB = 0.0;
    double dsEF = INFINITY;
    double vEF = 0.0;
    double dsEB = INFINITY;
    double vEB = 0.0;

    // iterate over targets
    for (auto &target : _input.targets) {

        // get target
        auto tar = &target;

        if (tar->lane == 0 && tar->ds >= 0.0 && tar->ds < dsEF) {
            dsEF = tar->ds;
            vEF = tar->v;
        } else if (tar->lane == 0 && tar->ds < 0.0 && tar->ds > dsEB) {
            dsEB = tar->ds;
            vEB = tar->v;
        } else if (tar->lane == 1 && tar->ds >= 0.0 && tar->ds < dsLF) {
            dsLF = tar->ds;
            vLF = tar->v;
        } else if (tar->lane == 1 && tar->ds < 0.0 && tar->ds > dsLB) {
            dsLB = tar->ds;
            vLB = tar->v;
        } else if (tar->lane == -1 && tar->ds >= 0.0 && tar->ds < dsRF) {
            dsRF = tar->ds;
            vRF = tar->v;
        } else if (tar->lane == -1 && tar->ds < 0.0 && tar->ds > dsRB) {
            dsRB = tar->ds;
            vRB = tar->v;
        }

    }

    // get values
    double v0 = _param.velocity.vComfort;
    double s0 = _param.follow.dsStopped;
    double T = _param.follow.timeHeadway;
    double v = _input.vehicle.v;
    double a = _param.velocity.a;
    double b = -_param.velocity.b;
    double bSafe = _param.laneChange.bSafe;
    double aThr = _param.laneChange.aThreshold;
    double p = _param.laneChange.politenessFactor;

    // instantiate result
    double sR, iR;
    double sL, iL;

    // calculate threshold dependent on following reaction TODO:
    // auto aThr = -std::min(0.0, agent_model::IDMOriginal(v, v0, v * _param.follow.thwMax, v - vEF, T, s0, a, b));

    // calculate lane change
    agent_model::MOBILOriginal(sR, iR, v, v0, T, s0, a, b, dsEF, vEF, dsRF, vRF, -dsEB, vEB, -dsRB, vRB, bSafe, aThr, p);
    agent_model::MOBILOriginal(sL, iL, v, v0, T, s0, a, b, dsEF, vEF, dsLF, vLF, -dsEB, vEB, -dsLB, vLB, bSafe, aThr, p);

    // TODO: only decide lane change, when lane is available
    if(sL > 0.999 && iL >= -0.5)
        _state.decisions.laneChange = 1;
    else if(sR > 0.999 && iR >= -0.5)
        _state.decisions.laneChange = -1;
    else
        _state.decisions.laneChange = 0;
}


void AgentModel::decisionLateralOffset() {

    _state.decisions.lateral.distance = INFINITY;
    _state.decisions.lateral.time = INFINITY;
    _state.decisions.lateral.value = 0.0;

}


void AgentModel::consciousVelocity() {

    using namespace std;

    // set max comfortable speed
    double vComf = _param.velocity.vComfort;
    _vel_horizon.setMaxVelocity(_param.velocity.vComfort);

    // some variables
    double dsLoc = -1.0 * INFINITY;
    double vLoc = INFINITY;

    // start for interval
    double s0 = _input.vehicle.s;
    double v0 = INFINITY;

    // unset the speed rules
    _vel_horizon.resetSpeedRule();

    // find last rule
    for (const auto &e : _input.signals) {

        // get speed limits
        if (e.type != agent_model::SignalType::SIGNAL_SPEED_LIMIT)
            continue;

        // speed
        auto v = e.value < 0 ? INFINITY : (double) e.value / 3.6;

        // check if closest rule
        if (e.ds < 0.0 && dsLoc < e.ds) {
            vLoc = v;
            dsLoc = e.ds;
        }

        // calculate end of interval
        double s1 = _input.vehicle.s + e.ds;

        // add rule to horizon
        if(s1 > s0)
            _vel_horizon.updateSpeedRuleInInterval(s0, s1, v0);

        s0 = s1;
        v0 = v;

    }

    // add rule to horizon
    _vel_horizon.updateSpeedRuleInInterval(s0, INFINITY, v0);

    // save local speed limit to state
    _memory.velocity = isinf(vLoc) ? _memory.velocity : vLoc;
    double vRule = _memory.velocity;

    // calculate local curve speed
    double kappaCurrent = isinf(_input.horizon.ds[1])
            ? 0.0 : agent_model::interpolate(0.0, _input.horizon.ds, _input.horizon.kappa, agent_model::NOH);
    double vCurve = max(0.0, sqrt(std::abs(_param.velocity.ayMax / kappaCurrent)));

    // iterate over horizon points
    for(unsigned int i = 0; i < agent_model::NOH; ++i) {

        // get position and speed
        auto s = _input.vehicle.s + _input.horizon.ds[i];
        auto v = max(0.0, sqrt(std::abs(_param.velocity.ayMax / _input.horizon.kappa[i])));

        // set speed
        _vel_horizon.updateContinuousPoint(s, v);

    }

    // sets the local speed
    _state.conscious.velocity.local = min(min(vComf, vRule), vCurve);

    // calculate interval
    double sI0 = _input.vehicle.s;
    double sI1 = sI0 + std::max(1.0, _input.vehicle.v * _param.velocity.thwMax);

    // calculate mean predictive velocity
    _state.conscious.velocity.prediction = _vel_horizon.mean(sI0, sI1, _param.velocity.deltaPred);

}


void AgentModel::consciousStop() {

    using namespace std;

    // add new signals
    agent_model::DecisionStopping signal = _state.decisions.signal;
    agent_model::DecisionStopping target = _state.decisions.target;
    agent_model::DecisionStopping destination = _state.decisions.destination;

    // check position and add stop point
    if(!std::isinf(signal.position))
        _stop_horizon.addStopPoint(signal.id, signal.position, signal.standingTime);

    if(!std::isinf(target.position))
        _stop_horizon.addStopPoint(target.id, target.position, target.standingTime);

    if(!std::isinf(destination.position))
        _stop_horizon.addStopPoint(destination.id, destination.position, destination.standingTime);

    // get stop
    auto stop = _stop_horizon.getNextStop();
    auto standing = false;

    // check standing
    if(!isinf(stop.ds)) {

        // is standing?
        standing = _input.vehicle.v < _param.stop.vStopped && stop.ds <= 0.5;

        // mark as stopped, TODO: mark all of them
        if(standing)
            _stop_horizon.stopped(stop.id, _state.simulationTime);

    }

    // default values
    _state.conscious.stop.ds = stop.ds;
    _state.conscious.stop.dsMax = stop.interval;
    _state.conscious.stop.standing = standing;

}


void AgentModel::consciousFollow() {

    // get pointer to targets
    auto t = _input.targets;

    // calculate net distance for following
    unsigned long im = agent_model::NOT;
    for (unsigned long i = 0; i < agent_model::NOT; ++i) {

        // ignore non-relevant targets (, ds < 0, other lane)
        if (t[i].id == 0 || std::isinf(t[i].ds) || t[i].ds < 0.0 || t[i].lane != 0)
            continue;

        // check if distance is smaller
        if (im == agent_model::NOT || t[im].ds > t[i].ds)
            im = i;

    }

    // instantiate distance and  velocity
    double ds = INFINITY, v = 0.0;
    if (im != agent_model::NOT) {
        ds = t[im].ds - t[im].size.length * 0.5 - _param.vehicle.size.length * 0.5 + _param.vehicle.pos.x;
        v = t[im].v;
    }

    // calculate if vehicle stands behind target vehicle
    bool standing = _input.vehicle.v < 1e-3 && v < 0.5 && ds <= _param.follow.dsStopped + 1e-2;

    // save distance and velocity
    _state.conscious.follow.distance = ds;
    _state.conscious.follow.velocity = v;
    _state.conscious.follow.standing = standing;

}


void AgentModel::consciousLaneChange() {

    if(_state.decisions.laneChange != 0 && !_lane_change_process_interval.isSet()) {

        // start process
        _lane_change_process_interval.setTimeInterval(_param.laneChange.time);
        _lane_change_process_interval.setScale(1.0 * _state.decisions.laneChange);

    }

    // calculate factor and limit
    _memory.laneChange.switchLane = 0;
    auto factor = _lane_change_process_interval.getScaledFactor();
    factor = std::max(-1.0, std::min(1.0, factor));

    // lane change has ended
    if(_lane_change_process_interval.getFactor() >= AM_CLOSE_TO_ONE) {

        // set lane change flag
        _memory.laneChange.switchLane = factor < 0.0 ? -1 : 1;

        // reset process
        _lane_change_process_interval.reset();
        _lane_change_process_interval.setScale(0.0);

    }

    // set factor, TODO: multi-lane change
    _state.conscious.lateral.paths[0].factor = (1.0 - std::abs(factor));
    _state.conscious.lateral.paths[1].factor = std::max(0.0, -factor);
    _state.conscious.lateral.paths[2].factor = std::max(0.0,  factor);

}


void AgentModel::consciousLateralOffset() {

    using namespace std;


    // check decision and set
    if(!isinf(_state.decisions.lateral.distance) || !isinf(_state.decisions.lateral.time)) {

        // save current offset
        _memory.lateral.offset = _input.vehicle.d;

        // reset
        _lateral_offset_interval.reset();

        // set intervals
        _lateral_offset_interval.setEndPosition(_input.vehicle.s + _state.decisions.lateral.distance);
        _lateral_offset_interval.setTimeInterval(_state.decisions.lateral.time);
        _lateral_offset_interval.setScale(_state.decisions.lateral.value);

    }

    // calculate factor and get scale
    auto factor = _lateral_offset_interval.getFactor();
    auto offset = _lateral_offset_interval.getScale();

    // set state
    _state.conscious.lateral.paths[0].offset = _memory.lateral.offset * (1.0 - factor) + offset * factor;

}


void AgentModel::consciousReferencePoints() {

    using namespace std;

    // get speed and offset
    auto v = _input.vehicle.v;

    // calculate reference points
    for (size_t i = 0; i < agent_model::NORP; ++i) {

        // get grid point
        double s = std::max(_param.steering.dsMin[i], v * _param.steering.thw[i]);

        // no interpolation possible (e.g. horizon ended) -> set horizon straight ahead
        if (isinf(_input.horizon.ds[1])) {

            // set all paths equal
            _state.conscious.lateral.paths[0].refPoints[i] = {s, 0.0, 0.0, 0.0};  // ego
            _state.conscious.lateral.paths[1].refPoints[i] = {s, 0.0, 0.0, 0.0};  // left
            _state.conscious.lateral.paths[2].refPoints[i] = {s, 0.0, 0.0, 0.0};  // right

            // next loop step
            continue;

        }

        // get lane offsets
        auto offR = agent_model::interpolate(s, _input.horizon.ds, _input.horizon.rightLaneOffset, agent_model::NOH, 2);
        auto offL = agent_model::interpolate(s, _input.horizon.ds, _input.horizon.leftLaneOffset, agent_model::NOH, 2);

        // interpolate angle and do the rotation math
        auto psi = agent_model::interpolate(s, _input.horizon.ds, _input.horizon.psi, agent_model::NOH, 2);
        double sn = sin(psi), cn = cos(psi);

        // get offset
        auto off = _state.conscious.lateral.paths[0].offset;

        // interpolate x and y
        auto x = agent_model::interpolate(s, _input.horizon.ds, _input.horizon.x, agent_model::NOH, 2) + _param.vehicle.pos.x - sn * off;
        auto y = agent_model::interpolate(s, _input.horizon.ds, _input.horizon.y, agent_model::NOH, 2) + _param.vehicle.pos.y + cn * off;

        // calculate reference points
        // TODO: calculate dx, dy (independent on change in speed => dv/dt = 0 to avoid influence of speed change in reference points)
        auto re = agent_model::DynamicPosition{x, y, 0.0, 0.0};
        auto rr = agent_model::DynamicPosition{x - sn * offR, y + cn * offR, 0.0, 0.0};
        auto rl = agent_model::DynamicPosition{x - sn * offL, y + cn * offL, 0.0, 0.0};

        // write data
        _state.conscious.lateral.paths[0].refPoints[i] = re;  // ego
        _state.conscious.lateral.paths[1].refPoints[i] = rr;  // right
        _state.conscious.lateral.paths[2].refPoints[i] = rl;  // left

    }

}


double AgentModel::subconsciousLateralControl() {

    // initialize reaction
    double reaction = 0.0;

    _state.aux[31] = 0.0;

    // iterate over reference points
    for (unsigned int i = 0; i < agent_model::NORP; ++i) {

        // parameters
        auto P = _param.steering.P[i];
        auto D = _param.steering.D[i];

        // iterate over control paths
        for (size_t j = 0; j < agent_model::NOCP; ++j) {

            // get reference points with derivatives
            auto x = _state.conscious.lateral.paths[j].refPoints[i].x;
            auto y = _state.conscious.lateral.paths[j].refPoints[i].y;
            auto dx = _state.conscious.lateral.paths[j].refPoints[i].dx;
            auto dy = _state.conscious.lateral.paths[j].refPoints[i].dy;

            // generate index for aux vector (theta and dtheta/dt are stored in aux)
            auto idx = 2 * (i * agent_model::NOCP + j);

            // calculate salvucci and gray and apply factor
            reaction += _state.conscious.lateral.paths[j].factor
                    * agent_model::SalvucciAndGray(x, y, dx, dy, P, D, _state.aux[idx + 0], _state.aux[idx + 1]);

            // save factored value
            _state.aux[31] += _state.conscious.lateral.paths[j].factor * _state.aux[idx];

        }

    }

    return reaction;

}


double AgentModel::subconsciousFollow() {

    using namespace std;

    // get values
    double vT = _state.conscious.follow.velocity;
    double ds = _state.conscious.follow.distance;
    double v0 = _state.conscious.velocity.local;
    double s0 = _param.follow.dsStopped;
    double T = _param.follow.timeHeadway;
    double TMax = _param.follow.thwMax;
    double v = _input.vehicle.v;

    // ignore when distance is inf
    if (std::isinf(ds))
        return 0.0;

    double v0T = std::max(10.0, v0);
    double vTT = std::min(v0T, std::max(5.0, vT));

    // calculate compensating time headway
    double TT = (s0 + T * vTT - (T * vTT * sqrt(vTT * vTT + v0T * v0T) * sqrt(vTT + v0T) * sqrt(v0T - vTT)) / (v0T * v0T)) / vTT;
    TT = max(0.0, min(T, TT));

    // scale down factor
    double f = agent_model::scaleInf(ds, v0 * TMax, vT * T);
    double fT = agent_model::scale(vT, 5.0, 0.0);

    // calculate reaction
    return agent_model::IDMFollowReaction(ds * f, vT, v, T - fT * TT, s0, _param.velocity.a, _param.velocity.b);

}


double AgentModel::subconsciousStop() {

    using namespace std;

    // get states
    double v = _input.vehicle.v;
    double ds = _state.conscious.stop.ds;
    double dsMax = _state.conscious.stop.dsMax;

    // get parameters
    double s0 = 2.0; // never set to 0.0 (this value is used to give IDM parameter s0 a value, its compensated though)
    double T = 1.2; // time headway (this is only to have a degressive behavior)
    double a = _param.velocity.a; // acceleration
    double b = _param.velocity.b; // deceleration

    // abort, when out of range
    if (ds > dsMax || isinf(dsMax))
        return 0.0;

    // apply s0
    ds += s0;
    dsMax += s0;

    // distance scaling (to have a smooth transition from uninfluenced to stopping)
    ds *= agent_model::scaleInf(ds, dsMax, s0, 1.0);

    // calculate reaction
    return agent_model::IDMFollowReaction(ds, 0.0, v, T, s0, a, b);

}


double AgentModel::subconsciousSpeed() {

    // scale parameter
    double deltaLoc = agent_model::scale(_state.conscious.velocity.local, 10.0, 2.0, 1.0) * 3.5 + 0.5;
    double deltaPred = agent_model::scale(_state.conscious.velocity.prediction, 10.0, 2.0, 1.0) * 3.5 + 0.5;

    // calculate reaction
    auto local = agent_model::IDMSpeedReaction(_input.vehicle.v, _state.conscious.velocity.local, deltaLoc);
    auto pred = agent_model::IDMSpeedReaction(_input.vehicle.v, _state.conscious.velocity.prediction, deltaPred);

    // return
    return _filter.value(std::max(local, pred));

}


double AgentModel::subconsciousStartStop() {

    // check for standing
    return (_state.conscious.stop.standing || _state.conscious.follow.standing)
           ? _param.stop.pedalDuringStanding : INFINITY;

}
