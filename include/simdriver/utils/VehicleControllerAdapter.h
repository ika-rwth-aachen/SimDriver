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
// Created by Jens Klimke on 2020-03-11.
//


#ifndef SIM_DRIVER_VEHICLECONTROLLERADAPTER_H
#define SIM_DRIVER_VEHICLECONTROLLERADAPTER_H

#include <simcore/ISynchronized.h>
#include <simtraffic/VehicleModel.h>
#include "PedalController.h"

class VehicleControllerAdapter : public sim::ISynchronized, public sim::traffic::Controller {

public:

    sim::traffic::VehicleModel vehicle;
    sim::traffic::PedalController pedal;
    sim::traffic::Controller steer;

    VehicleControllerAdapter() = default;
    ~VehicleControllerAdapter() override = default;


    void initialize(double initTime) override {

        // reset controllers
        pedal.reset();
        steer.reset();

    }


    void step(double simTime, double dt) override {

        // run controller steps
        pedal.step(dt);
        steer.step(dt);

        // run vehicle step
        vehicle.step(dt);

    }


    void terminate(double simTime) override {



    }

};


#endif //SIM_DRIVER_VEHICLECONTROLLERADAPTER_H
