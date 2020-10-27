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
// Created by Jens Klimke on 2020-10-27.
//


#ifndef SIMCORE_PEDAL_CONTROLLER_H
#define SIMCORE_PEDAL_CONTROLLER_H

#include "Controller.h"
#include "Filter.h"

namespace sim::traffic {


    class PedalController : public Controller<double> {

        double targetPosition;
        double targetVelocity;
        double targetAcceleration;

        double actualPosition;
        double actualVelocity;

        Filter<double> acceleration;

        PIDParameters positionParameters;
        PIDParameters velocityParameters;

        PIDMemory positionMemory;
        PIDMemory velocityMemory;

    public:

        void reset() override {
        }

        void step(double dt) override {

        }

    };


}

#endif // SIMCORE_PEDAL_CONTROLLER_H
