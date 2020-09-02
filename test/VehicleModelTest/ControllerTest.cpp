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
// Created by Jens Klimke on 2019-03-22.
//

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma ide diagnostic ignored "cert-err58-cpp"

#include <gtest/gtest.h>
#include <simcore/functions.h>
#include <simcore/traffic/BasicSimulation.h>
#include <VehicleModel/VehicleModel.h>
#include <VehicleModel/PrimaryController.h>

class ControllerTest : public ::testing::Test, public VehicleModel, public BasicSimulation, public sim::IComponent {

public:

    PrimaryController pCtrl;
    PrimaryController sCtrl;

    VehicleModel::Input      *input = nullptr;
    VehicleModel::Parameters *param = nullptr;
    VehicleModel::State      *state = nullptr;

    double accTarget = 0.0;
    double dPsiTarget = 0.0;
    double kappaTarget = 0.0;
    double pedalTarget = INFINITY;

    ControllerTest() = default;
    ~ControllerTest() override = default;

    void SetUp() override {

        // create simulation
        create(100.0, 0.01, false);

        // add this to loop
        addComponent(this);

        // get pointers
        input = this->getInput();
        param = this->getParameters();
        state = this->getState();

        // setup pedal controller
        pCtrl.setVariables(&state->a, &accTarget, &input->pedal, &pedalTarget);
        pCtrl.setRange(-1.0, 1.0, 1.0);

        // setup pedal controller
        sCtrl.setVariables(&state->dPsi, &dPsiTarget, &input->steer);
        sCtrl.setRange(-1.0, 1.0, 1.0);

        // setup acceleration controller
        pCtrl.setParameters(2.0, 0.1, 0.0);
        sCtrl.setParameters(1.0, 0.0, 0.0);

        // reset the controller
        pCtrl.reset();
        sCtrl.reset();

    }

    bool step(double simTime) override {

        auto dt = timeStep(simTime);

        // controller step
        pCtrl.step(dt);
        sCtrl.step(dt);

        // vehicle step
        VehicleModel::step(dt);

        return true;

    }

    void initialize(double initTime) override {

        IComponent::initializeTimer(initTime);

    }

    void terminate(double simTime) override {

    }

};


TEST_F(ControllerTest, ControlledDirectValue) {

    pedalTarget = 0.89;

    // set initial speed
    double vInit = 0.0;
    state->v = vInit;

    // set pedal and steering value
    input->pedal = -1.0;
    input->steer =  0.0;
    input->slope =  0.0;

    // run simulation
    run();

    // this values need to fit
    EXPECT_NEAR(dPsiTarget,  state->dPsi,  1e-6);
    EXPECT_NEAR(pedalTarget, input->pedal, 1e-6);

}


TEST_F(ControllerTest, Error) {

    accTarget = INFINITY;

    // set initial speed
    double vInit = 0.0;
    state->v = vInit;

    // set pedal and steering value
    input->pedal =  0.0;
    input->steer =  0.0;
    input->slope =  0.0;

    // run simulation
    EXPECT_THROW(run(), std::runtime_error);

}



TEST_F(ControllerTest, ControlledYawRate) {

    // set target values
    accTarget = 0.0;
    dPsiTarget = 0.1;

    // set initial speed
    double vInit = 20.0;
    state->v = vInit;

    // set pedal and steering value
    input->pedal = 0.0;
    input->steer = 0.1;
    input->slope = 0.0;

    // run simulation
    run();

    // this values need to fit
    EXPECT_NEAR(vInit,      state->v,    1.0);
    EXPECT_NEAR(accTarget,  state->a,    1e-6);
    EXPECT_NEAR(dPsiTarget, state->dPsi, 1e-6);

}


TEST_F(ControllerTest, ControlledCurvature) {

    // set target values
    accTarget   = 0.0;
    kappaTarget = 0.01;

    // update controller
    sCtrl.setParameters(10.0, 0.0, 0.0);
    sCtrl.setVariables(&state->kappa, &kappaTarget, &input->steer);

    // reset the controller
    sCtrl.reset();

    // set initial state
    state->delta =  0.0;
    state->v     = 10.0;

    // run simulation
    run();

    EXPECT_NEAR(accTarget,   state->a,     1e-6);
    EXPECT_NEAR(kappaTarget, state->kappa, 1e-6);

}

#pragma clang diagnostic pop