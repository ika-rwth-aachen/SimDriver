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

#include <simcore/Model.h>
#include <VehicleModel/VehicleModel.h>
#include <VehicleModel/PrimaryController.h>

class VehicleControllerAdapter : public sim::Model {

public:

    VehicleModel vehicle;
    PrimaryController pedal;
    PrimaryController steer;

    VehicleControllerAdapter() = default;
    ~VehicleControllerAdapter() override = default;


    void initialize(double initTime) override {

        // initialize timer
        Model::initialize(initTime);
        Model::initializeTimer(initTime);

        // reset controllers
        pedal.reset();
        steer.reset();

    }


    bool step(double simTime) override {

        if(!Model::step(simTime))
            return false;

        // get time step size
        auto dt = timeStep(simTime);

        // run steps
        pedal.step(dt);
        steer.step(dt);
        vehicle.step(dt);

        return true;

    }


    void terminate(double simTime) override {

        Model::terminate(simTime);

    }

};


#endif //SIM_DRIVER_VEHICLECONTROLLERADAPTER_H
