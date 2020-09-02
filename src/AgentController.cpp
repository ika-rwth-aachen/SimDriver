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
// Created by Jens Klimke on 2020-06-05.
//


#include "AgentController.h"
#include <algorithm>

bool AgentController::step(double dt) {

    // calculate gained error
    auto dyP = k_P * (*targetP - *actualP);
    auto dyI = k_I * (*targetI - *actualI);
    auto dyD = k_D * (*targetD - *actualD);
    auto dyA = k_A * _filter.value(*preValue);
    auto dyO = *offset;

    // limit change of controller
    *output = std::min(this->range[1], std::max(this->range[0], dyP + dyI + dyD + dyA + dyO));

    return true;

}


void AgentController::setOffset(double *off) {

    this->offset = off;

}


void AgentController::setAnticipation(double *value, double k, unsigned int filterLen) {

    this->preValue = value;
    this->k_A = k;

    // init filter
    this->_filter.init(filterLen);

}


void AgentController::setProportionalCompensation(double *actual, double *target, double k) {

    this->actualP = actual;
    this->targetP = target;
    this->k_P = k;

}


void AgentController::setIntegralCompensation(double *actual, double *target, double k) {

    this->actualI = actual;
    this->targetI = target;
    this->k_I = k;

}


void AgentController::setDerivativeCompensation(double *actual, double *target, double k) {

    this->actualD = actual;
    this->targetD = target;
    this->k_D = k;

}


void AgentController::setOutput(double *out, double max, double min) {

    this->output = out;
    this->range[0] = min;
    this->range[1] = max;

}