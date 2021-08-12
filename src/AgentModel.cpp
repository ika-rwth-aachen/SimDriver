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
    //consciousLaneChange();              // open
	_state.conscious.lateral.paths[0].factor = 1;
    consciousLateralOffset();           // done: Test 7.3
    consciousReferencePoints();         // done: Test 7.1, 7.2, 7.3, 7.4

    // apply injection for conscious states
    APPLY(&this->_state.conscious)

    // calculate speed reaction
    double rSpeed  = subconsciousSpeed();          // done: Test 2.1, 2.2, 2.3, 3.1, 3.2, 3.6
    double rStop   = subconsciousStop();           // done: Test 3.3, 3.6
    double rFollow = subconsciousFollow();         // done: Test 3.4, 3.5, 3.6
    double pedal   = subconsciousStartStop();      // done: Test 1.1, 1.2
    double kappa   = subconsciousLateralControl() ;//+ 3./6 * (_input.horizon.kappa[0]+_input.horizon.kappa[1]+_input.horizon.kappa[2]+_input.horizon.kappa[3]+_input.horizon.kappa[4]+_input.horizon.kappa[5]); // done: Test 6.1, 6.2, 6.3, 6.4
    //std::cout << "current hor kappa: " << _input.horizon.kappa[0] << "\n";
    // calculate resulting acceleration
    double aRes = _param.velocity.a * (1.0 - rSpeed - rStop - rFollow);
    //std::cout << "reactions: " << rSpeed << ","  << "," << rStop << "," << rFollow << "\n";

    // set desired values
    _state.subconscious.a     = std::min(std::max(-10.0, aRes), 10.0);           // done: Test 1.3
    // d penalty:
    double pen = int((_input.vehicle.d>0) - (_input.vehicle.d<0) ) * _input.vehicle.d *_input.vehicle.d * 0.25;
    _state.subconscious.kappa = kappa - pen;          // done: Test 4.1, 4.2
    _state.subconscious.pedal = pedal;          // done: Test 1.4
    _state.subconscious.steering = INFINITY;

    // apply injection for sub-conscious states
    APPLY(&this->_state.subconscious)

    // save values to memory
    _memory.vehicle.s = _input.vehicle.s;

}


void AgentModel::decisionProcessStop() {

    // unset decision
    for(auto &e : _state.decisions.stopping) {
        e.id = std::numeric_limits<unsigned int>::max();
        e.position = INFINITY;
        e.standingTime = INFINITY;
    }


    // add new signals
    unsigned int i = 0;
    for(auto &e : _input.signals) {

        // not a stop sign
        if(e.type != agent_model::SignalType::SIGNAL_STOP 
        && e.type != agent_model::SignalType::SIGNAL_TLS
        && e.type != agent_model::SignalType::SIGNAL_YIELD
        && e.type != agent_model::SignalType::SIGNAL_PRIORITY)
            continue;

        // calculate net distance
        auto ds = e.ds - _param.stop.dsGap + _param.vehicle.pos.x - _param.vehicle.size.length * 0.5;

        // do not add when distance is too large
        if(ds > _input.vehicle.v * _param.stop.TMax && ds > _param.stop.dsMax)
            continue;

        // do not add a yield sign if no target is not relevant
        double tJunctionEgo = ds / _input.vehicle.v;
    
        // find relevant targets
        int ii = 0;
        for(auto &t : _input.targets)
        {
            // ignore unset targets
            if(t.id == 0) continue;

            // wait if target is oncomming when turning left
            if(e.type == agent_model::SignalType::SIGNAL_PRIORITY
            && _input.vehicle.maneuver == agent_model::Maneuver::TURN_LEFT
            && t.priority == agent_model::TargetPriority::TARGET_ON_PRIORITY_LANE
            && t.dsIntersection <= t.v * _param.stop.TMax) // make sure that dsIntersection is set to inf when target is leaving intersection
            {
                ii++;
                continue;
            }

            // ego has a yield sign in front
            if(e.type == agent_model::SignalType::SIGNAL_YIELD) 
            {
                // ignore if target has no prority
                if(t.priority == agent_model::TargetPriority::TARGET_PRIORITY_NOT_SET 
                || t.priority == agent_model::TargetPriority::TARGET_ON_GIVE_WAY_LANE)
                    continue;

                // ignore if target is too far away from junction
                // double tJunctionTarget = t.dsIntersection / t.v;
                // TODO: do something with both time variables...
                // TODO: maybe check if target will have passed intersection
                
                // Mark as relevant if target is on intersection
                if(t.priority == agent_model::TargetPriority::TARGET_ON_INTERSECTION)
                {
                    ii++;
                    continue;
                }

                // 
                if(t.priority == agent_model::TargetPriority::TARGET_ON_PRIORITY_LANE
                && t.dsIntersection <= t.v * _param.stop.TMax)
                {
                    ii++;
                    continue;
                }                
            }

            // check if driver has to wait when turning left
            if(e.type == agent_model::SignalType::SIGNAL_PRIORITY 
            && _input.vehicle.maneuver == agent_model::Maneuver::TURN_LEFT
            ) 
            {
                continue;
            }

        }
        if(ii==0) continue;

     
        // add stop point
        _state.decisions.stopping[i].id = e.id;
        _state.decisions.stopping[i].position = _input.vehicle.s + ds;
        _state.decisions.stopping[i].standingTime = _param.stop.tSign;

        // increment
        i++;

    }

}


void AgentModel::decisionLaneChange() {


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

    // get stop
    agent_model::StopHorizon::StopPoint stop;
    auto standing = false;
    // add new signals
    for(auto &e : _state.decisions.stopping) {

        // check position and add stop point
        if(!std::isinf(e.position)) {
            _stop_horizon.addStopPoint(e.id, e.position, e.standingTime);
            stop = _stop_horizon.getNextStop();
        }

    }

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

        // get lane widths
        auto we = agent_model::interpolate(s, _input.horizon.ds, _input.horizon.egoLaneWidth, agent_model::NOH, 2);
        auto wr = agent_model::interpolate(s, _input.horizon.ds, _input.horizon.rightLaneWidth, agent_model::NOH, 2);
        auto wl = agent_model::interpolate(s, _input.horizon.ds, _input.horizon.leftLaneWidth, agent_model::NOH, 2);

        // get lane offsets
        auto offR = -0.5 * (we + wr);
        auto offL = 0.5 * (we + wl);

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
        _state.conscious.lateral.paths[1].refPoints[i] = rr;  // left
        _state.conscious.lateral.paths[2].refPoints[i] = rl;  // right

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
