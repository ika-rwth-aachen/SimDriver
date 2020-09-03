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

    // set inputs
    input.pedal = 0.0;
    input.steer = 0.0;

}


void VehicleModel::step(double timeStepSize) {

    // short cuts
    auto &dt = timeStepSize;
    auto p   = &parameters;
    auto s  = &state;
    auto i   = &input;

    // calculate wheel steer angle and curvature
    s->delta = p->maxWheelAngle * i->steer;
    s->kappa = s->delta / p->wheelBase;

    // calculate distance and velocity
    s->ds = std::max(0.0, s->v * dt + 0.5 * s->a * dt * dt);
    s->v = std::max(0.0, s->v + s->a * dt);

    // calculate position
    s->s += s->ds;
    s->position.x += cos(s->psi) * s->ds - sin(s->psi) * s->ds * p->sideDrift;
    s->position.y += sin(s->psi) * s->ds + cos(s->psi) * s->ds * p->sideDrift;

    // calculate yaw rate and yaw angle
    s->dPsi = s->v * s->kappa;
    s->psi += s->dPsi * dt;

    // throttle value and brake value
    auto throttle = std::max( 0.0, std::min(1.0, i->pedal * (1.0 - p->idlePedal) + p->idlePedal));
    auto brake    = std::max(-1.0, std::min(0.0, i->pedal));

    // calculate accelerations (external, drive, brake)
    auto aLong = p->longExternalAcc[0] + p->longExternalAcc[1] * s->v + p->longExternalAcc[2] * s->v * s->v;
    auto aAcc = throttle * std::min(p->maxDriveAcc, p->maxRelDrivePower / s->v);
    auto aDec = brake * p->maxBrakeAcc;

    // calculate acceleration
    s->a = std::max(-G_ACC, std::min(G_ACC, -aDec + aAcc - aLong));

    // unset acceleration, when standing
    if(s->v == 0.0 && s->a < 0.0)
        s->a = 0.0;

}

VehicleModel::Input * VehicleModel::getInput() {

    return &this->input;

}

const VehicleModel::Input * VehicleModel::getInput() const {

    return &this->input;

}

VehicleModel::State *VehicleModel::getState()  {

    return &this->state;

}

const VehicleModel::State *VehicleModel::getState() const  {

    return &this->state;

}

VehicleModel::Parameters *VehicleModel::getParameters() {

    return &this->parameters;

}

const VehicleModel::Parameters *VehicleModel::getParameters() const {

    return &this->parameters;

}