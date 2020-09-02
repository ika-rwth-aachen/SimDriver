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
// Created by Jens Klimke on 2019-04-15.
//

#include <stdexcept>
#include <algorithm>
#include <cmath>
#include "PrimaryController.h"

bool PrimaryController::step(double timeStepSize) {

    auto &dt = timeStepSize;

    // check if value is set
    if(_target == nullptr)
        return false;

    // calculate error (P)
    auto _u = *_target - *_value;

    if(std::isinf(_u))
        throw std::runtime_error("Input value is not finite.");

    // calculate integral (I)
    in += (u + _u) * dt;

    // calculate derivative (D)
    auto der = _reset ? 0.0 : (_u - u) / dt;
    u = _u;

    // unset reset flag
    _reset = false;

    // calculate controller change
    auto dy = k_P * u + k_I * in + k_D * der;

    // apply desired controller state
    if(_offset != nullptr && !std::isinf(*_offset)) {

        // change controller value
        dy = (*_offset - *_y) * o_P;

        // reset controller
        reset();

    }

    // limit change of controller
    dy = std::min(_maxChange, std::max(-_maxChange, dy));

    // integrate
    *_y = std::max(_range[0], std::min(_range[1], *_y + dy * dt));

    return true;

}

void PrimaryController::reset() {

    in = 0.0;
    _reset = true;

}


void PrimaryController::setVariables(double *value, double *target, double *output, double *offset) {

    this->_value = value;
    this->_target = target;
    this->_offset = offset;
    _y = output;

}


void PrimaryController::setParameters(double k_p, double k_i, double k_d, double o_p) {

    k_P = k_p;
    k_I = k_i;
    k_D = k_d;
    o_P = o_p;

}

void PrimaryController::setRange(double lower, double upper, double maxChange) {

    this->_range[0] = lower;
    this->_range[1] = upper;
    this->_maxChange = maxChange;

}