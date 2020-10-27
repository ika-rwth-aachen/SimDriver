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
#include <limits>

#include <SimDriver/Behavior.h>
#include <SimDriver/model_collection.h>

#ifndef NEG_INFINITY
#define NEG_INFINITY (-1.0 * INFINITY)
#endif

#if WITH_INJECTION
#include <Injection/Injection.h>
#define APPLY(PNTR) { sim_driver::injection::Interface::applyAll(PNTR); sim_driver::injection::Interface::resetAll(PNTR); }
#else
#define APPLY(PNTR)
#endif

static const double AM_CLOSE_TO_ONE = 0.999999999;

void Behavior::init() {

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


void Behavior::step(double simulationTime) {

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
    _state.subconscious.a     = aRes;           // done: Test 1.3
    _state.subconscious.kappa = kappa;          // done: Test 4.1, 4.2
    _state.subconscious.pedal = pedal;          // done: Test 1.4
    _state.subconscious.steering = INFINITY;

    // apply injection for sub-conscious states
    APPLY(&this->_state.subconscious)

    // save values to memory
    _memory.vehicle.s = _input.vehicle.s;

}


void Behavior::decisionProcessStop() {

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
        if(e.type != sim_driver::SignalType::SIGNAL_STOP && e.type != sim_driver::SignalType::SIGNAL_TLS)
            continue;

        // calculate net distance
        auto ds = e.ds - _param.stop.dsGap + _param.vehicle.pos.x - _param.vehicle.size.length * 0.5;

        // do not add when distance is too large
        if(ds > _input.vehicle.v * _param.stop.TMax && ds > _param.stop.dsMax)
            continue;

        // add stop point
        _state.decisions.stopping[i].id = e.id;
        _state.decisions.stopping[i].position = _input.vehicle.s + ds;
        _state.decisions.stopping[i].standingTime = _param.stop.tSign;

        // increment
        i++;

    }

}


void Behavior::decisionLaneChange() {

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
    // auto aThr = -std::min(0.0, sim_driver::IDMOriginal(v, v0, v * _param.follow.thwMax, v - vEF, T, s0, a, b));

    // calculate lane change
    sim_driver::models::MOBILOriginal(sR, iR, v, v0, T, s0, a, b, dsEF, vEF, dsRF, vRF, -dsEB, vEB, -dsRB, vRB, bSafe, aThr, p);
    sim_driver::models::MOBILOriginal(sL, iL, v, v0, T, s0, a, b, dsEF, vEF, dsLF, vLF, -dsEB, vEB, -dsLB, vLB, bSafe, aThr, p);

    // TODO: only decide lane change, when lane is available
    if(sL > 0.999 && iL >= -0.5)
        _state.decisions.laneChange = 1;
    else if(sR > 0.999 && iR >= -0.5)
        _state.decisions.laneChange = -1;
    else
        _state.decisions.laneChange = 0;

}


void Behavior::decisionLateralOffset() {

    _state.decisions.lateral.distance = INFINITY;
    _state.decisions.lateral.time = INFINITY;
    _state.decisions.lateral.value = 0.0;

}


void Behavior::consciousVelocity() {

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
        if (e.type != sim_driver::SignalType::SIGNAL_SPEED_LIMIT)
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

    // calculate curvature
    double kappaCurrent = isinf(_input.horizon.ds[1])
            ? 0.0
            : sim_driver::math::interpolate(0.0, _input.horizon.ds, _input.horizon.kappa, sim_driver::NOH);

    // calculate velocity in curve
    double vCurve = max(0.0, sqrt(std::abs(_param.velocity.ayMax / kappaCurrent)));

    // iterate over horizon points
    for(unsigned int i = 0; i < sim_driver::NOH; ++i) {

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


void Behavior::consciousStop() {

    using namespace std;

    // add new signals
    for(auto &e : _state.decisions.stopping) {

        // check position and add stop point
        if(!std::isinf(e.position))
            _stop_horizon.addStopPoint(e.id, e.position, e.standingTime);

    }

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


void Behavior::consciousFollow() {

    // calculate net distance for following
    unsigned long im = sim_driver::NOT;
    for (unsigned long i = 0; i < sim_driver::NOT; ++i) {

        // ignore non-relevant targets (, ds < 0, other lane)
        if (std::isinf(_input.targets[i].ds) || _input.targets[i].ds < 0.0 || _input.targets[i].lane != 0)
            continue;

        // check if distance is smaller
        if (im == sim_driver::NOT || _input.targets[im].ds > _input.targets[i].ds)
            im = i;

    }

    // instantiate distance and  velocity
    double ds = INFINITY, v = 0.0;
    if (im != sim_driver::NOT) {
        ds = _input.targets[im].ds - _input.targets[im].size.length * 0.5 - _param.vehicle.size.length * 0.5 + _param.vehicle.pos.x;
        v = _input.targets[im].v;
    }

    // calculate if vehicle stands behind target vehicle
    bool standing = _input.vehicle.v < 1e-3 && v < 0.5 && ds <= _param.follow.dsStopped + 1e-2;

    // save distance and velocity
    _state.conscious.follow.distance = ds;
    _state.conscious.follow.velocity = v;
    _state.conscious.follow.standing = standing;

}


void Behavior::consciousLaneChange() {

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


void Behavior::consciousLateralOffset() {

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


void Behavior::consciousReferencePoints() {

    using namespace std;
    using namespace sim_driver::math;

    // get speed and offset
    auto v = _input.vehicle.v;

    // calculate reference points
    for (size_t i = 0; i < sim_driver::NORP; ++i) {

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
        auto we = interpolate(s, _input.horizon.ds, _input.horizon.egoLaneWidth, sim_driver::NOH, 2);
        auto wr = interpolate(s, _input.horizon.ds, _input.horizon.rightLaneWidth, sim_driver::NOH, 2);
        auto wl = interpolate(s, _input.horizon.ds, _input.horizon.leftLaneWidth, sim_driver::NOH, 2);

        // get lane offsets
        auto offR = -0.5 * (we + wr);
        auto offL = 0.5 * (we + wl);

        // interpolate angle and do the rotation math
        auto psi = interpolate(s, _input.horizon.ds, _input.horizon.psi, sim_driver::NOH, 2);
        double sn = sin(psi), cn = cos(psi);

        // get offset
        auto off = _state.conscious.lateral.paths[0].offset;

        // interpolate x and y
        auto x = interpolate(s, _input.horizon.ds, _input.horizon.x, sim_driver::NOH, 2) + _param.vehicle.pos.x - sn * off;
        auto y = interpolate(s, _input.horizon.ds, _input.horizon.y, sim_driver::NOH, 2) + _param.vehicle.pos.y + cn * off;

        // calculate reference points
        // TODO: calculate dx, dy (independent on change in speed => dv/dt = 0 to avoid influence of speed change in reference points)
        auto re = sim_driver::DynamicPosition{x, y, 0.0, 0.0};
        auto rr = sim_driver::DynamicPosition{x - sn * offR, y + cn * offR, 0.0, 0.0};
        auto rl = sim_driver::DynamicPosition{x - sn * offL, y + cn * offL, 0.0, 0.0};

        // write data
        _state.conscious.lateral.paths[0].refPoints[i] = re;  // ego
        _state.conscious.lateral.paths[1].refPoints[i] = rr;  // left
        _state.conscious.lateral.paths[2].refPoints[i] = rl;  // right

    }

}


double Behavior::subconsciousLateralControl() {

    using namespace sim_driver::models;

    // initialize reaction
    double reaction = 0.0;

    _state.aux[31] = 0.0;

    // iterate over reference points
    for (unsigned int i = 0; i < sim_driver::NORP; ++i) {

        // parameters
        auto P = _param.steering.P[i];
        auto D = _param.steering.D[i];

        // iterate over control paths
        for (size_t j = 0; j < sim_driver::NOCP; ++j) {

            // get reference points with derivatives
            auto x = _state.conscious.lateral.paths[j].refPoints[i].x;
            auto y = _state.conscious.lateral.paths[j].refPoints[i].y;
            auto dx = _state.conscious.lateral.paths[j].refPoints[i].dx;
            auto dy = _state.conscious.lateral.paths[j].refPoints[i].dy;

            // generate index for aux vector (theta and dtheta/dt are stored in aux)
            auto idx = 2 * (i * sim_driver::NOCP + j);

            // calculate salvucci and gray and apply factor
            reaction += _state.conscious.lateral.paths[j].factor
                    * SalvucciAndGray(x, y, dx, dy, P, D, _state.aux[idx + 0], _state.aux[idx + 1]);

            // save factored value
            _state.aux[31] += _state.conscious.lateral.paths[j].factor * _state.aux[idx];

        }

    }

    return reaction;

}


double Behavior::subconsciousFollow() {

    using namespace std;
    using namespace sim_driver::math;
    using namespace sim_driver::models;

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
    double f = scaleInf(ds, v0 * TMax, vT * T);
    double fT = scale(vT, 5.0, 0.0);

    // calculate reaction
    return IDMFollowReaction(ds * f, vT, v, T - fT * TT, s0, _param.velocity.a, _param.velocity.b);

}


double Behavior::subconsciousStop() {

    using namespace std;
    using namespace sim_driver::math;
    using namespace sim_driver::models;

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
    ds *= scaleInf(ds, dsMax, s0, 1.0);

    // calculate reaction
    return IDMFollowReaction(ds, 0.0, v, T, s0, a, b);

}


double Behavior::subconsciousSpeed() {

    using namespace sim_driver::math;
    using namespace sim_driver::models;

    // scale parameter
    double deltaLoc = scale(_state.conscious.velocity.local, 10.0, 2.0, 1.0) * 3.5 + 0.5;
    double deltaPred = scale(_state.conscious.velocity.prediction, 10.0, 2.0, 1.0) * 3.5 + 0.5;

    // calculate reaction
    auto local = IDMSpeedReaction(_input.vehicle.v, _state.conscious.velocity.local, deltaLoc);
    auto pred = IDMSpeedReaction(_input.vehicle.v, _state.conscious.velocity.prediction, deltaPred);

    // return
    return _filter.value(std::max(local, pred));

}


double Behavior::subconsciousStartStop() {

    // check for standing
    return (_state.conscious.stop.standing || _state.conscious.follow.standing)
           ? _param.stop.pedalDuringStanding : INFINITY;

}
