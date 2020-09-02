// Copyright (c) 2019-2020 Jens Klimke <jens.klimke@rwth-aachen.de>
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
// Created by Jens Klimke on 2019-03-20.
//

#include <cmath>
#include <algorithm>
#include "VehicleModel.h"

#ifndef G_ACC
#define G_ACC 9.81
#endif

#ifndef RHO_AIR
#define RHO_AIR 1.2041
#endif


void VehicleModel::reset() {

    // set initial states
    state.s        = 0.0;
    state.v        = 0.0;
    state.psi      = 0.0;
    state.position = {0.0, 0.0};

    // set calculated states
    state.ds    = 0.0;
    state.a     = 0.0;
    state.dPsi  = 0.0;
    state.delta = 0.0;
    state.kappa = 0.0;
    state.ay    = 0.0;
    state.force = 0.0;

    // set inputs
    input.slope = 0.0;
    input.pedal = 0.0;
    input.steer = 0.0;

}


bool VehicleModel::step(double timeStepSize) {

    // short cuts
    auto &dt = timeStepSize;
    auto p   = &parameters;
    auto st  = &state;

    // calculate wheel steer angle and curvature
    st->delta = p->steerTransmission * input.steer;
    st->kappa = st->delta / p->wheelBase;

    // calculate distance and velocity
    st->ds = std::max(0.0, st->v * dt + 0.5 * st->a * dt * dt);
    st->v = std::max(0.0, st->v + st->a * dt);

    // calculate position
    st->s += st->ds;
    st->position.x += cos(st->psi) * st->ds;
    st->position.y += sin(st->psi) * st->ds;

    // calculate yaw rate and yaw angle
    st->dPsi = st->v * st->kappa;
    st->psi += st->dPsi * dt;

    // squared velocity
    auto v2 = st->v * st->v;

    // coefficients
    auto airCoeff = 0.5 * RHO_AIR * p->cwA;
    auto rollCoeff = p->rollCoefficient[0] + p->rollCoefficient[1] * st->v + p->rollCoefficient[2] * v2;

    // limit power and gas pedal
    auto throttle = std::max(input.pedal, 0.0) * (1.0 - p->idle) + p->idle;

    // calculate accelerations
    auto aGround = cos(input.slope) * G_ACC;
    auto aAir   = airCoeff * v2 / p->mass;
    auto aRoll  = rollCoeff * aGround;
    auto aSlope = sin(input.slope) * G_ACC;
    auto aBrake = aGround * std::min(input.pedal, 0.0);

    // calculate smooth force curve
    double F0 = p->forceMax;
    double F1 = p->powerMax * 0.1;  // / 10 m/s (low speed boundary)
    double _x = st->v * 0.1;        // / 10 m/s (low speed boundary)

    // calculate drive force
    st->force = _x < 1.0
            ? (F0 + _x * _x * (4.0 * F1 -  3.0 * F0) + _x * _x * _x * (2.0 * F0 - 3.0 * F1))    // low speed
            : p->powerMax / st->v;                                                              // high speed

    // calculate acceleration
    st->a  = -aRoll - aAir - aSlope + aBrake + throttle * st->force / p->mass;
    st->ay = st->kappa * st->v * st->v;

    // unset acceleration, when standing
    if(st->v == 0.0 && st->a < 0.0)
        st->a = 0.0;

    return true;

}

VehicleModel::Input * VehicleModel::getInput() {

    return &this->input;

}

VehicleModel::State *VehicleModel::getState()  {

    return &this->state;

}

VehicleModel::Parameters *VehicleModel::getParameters() {

    return &this->parameters;

}

const VehicleModel::Input * VehicleModel::getInput() const {

    return &this->input;

}

const VehicleModel::State *VehicleModel::getState() const  {

    return &this->state;

}

const VehicleModel::Parameters *VehicleModel::getParameters() const {

    return &this->parameters;

}