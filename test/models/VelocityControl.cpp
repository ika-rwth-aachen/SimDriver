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
// Created by Jens Klimke on 2019-03-24.
// Contributors:
//
// MC_VelocityControl.cpp

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma ide diagnostic ignored "cert-err58-cpp"

#include <gtest/gtest.h>
#include <SimDriver/model_collection.h>
#include <cmath>

using namespace sim_driver::models;

const static double inf = INFINITY;

struct DriverVel {

    double v;
    double vDes;
    double delta;

    double dsStep[2];
    double vStep[2];
    double deltaStep;
    double TMax;

    double reaction;

};


struct ModelTestVelocity : public testing::Test, public testing::WithParamInterface<DriverVel> {

    ModelTestVelocity() = default;

};


TEST_P(ModelTestVelocity, IDMFreeTest) {

    auto as = GetParam();
    double a = 1.0 - speedReaction(as.v, as.vDes, as.delta, as.vStep, as.dsStep, as.TMax, as.deltaStep);

    // check acceleration
    EXPECT_NEAR(as.reaction, a, 1e-4);

}


INSTANTIATE_TEST_CASE_P(Free, ModelTestVelocity, testing::Values(

        //          v     vDes  de1  ds   vs   de2  T    r
        DriverVel{0.0, 0.0, 0.0, {inf, inf}, {0.0, 0.0}, 0.0, 0.0, -1.0},     //  0. all values are zero
        DriverVel{0.0, 0.0, 4.0, {inf, inf}, {0.0, 0.0}, 0.0, 0.0, -1.0},     //  1. all values except delta are zero
        DriverVel{0.0, inf, inf, {inf, inf}, {0.0, 0.0}, 0.0, 0.0, 1.0},     //  2. vDes and delta is inf
        DriverVel{0.0, inf, 4.0, {inf, inf}, {0.0, 0.0}, 0.0, 0.0, 1.0},     //  3. vDes is inf
        DriverVel{1e9, inf, 4.0, {inf, inf}, {0.0, 0.0}, 0.0, 0.0, 1.0},     //  4. v is very high, vDes is inf
        DriverVel{1e9, 0.0, 4.0, {inf, inf}, {0.0, 0.0}, 0.0, 0.0, -1.0},     //  5. v is very high

        DriverVel{0.0, 10.0, 4.0, {inf, inf}, {0.0, 0.0}, 0.0, 0.0, 1.0},     //  5. full reaction
        DriverVel{0.0, 10.0, 4.0, {inf, inf}, {0.0, 0.0}, 0.0, 0.0, 1.0},     //  6. full reaction
        DriverVel{0.0, 100.0, 4.0, {inf, inf}, {0.0, 0.0}, 0.0, 0.0, 1.0},     //  7. full reaction
        DriverVel{10.0, 0.0, 4.0, {inf, inf}, {0.0, 0.0}, 0.0, 0.0, -1.0},     //  8. full negative reaction

        DriverVel{10.0, 20.0, 0.0, {inf, inf}, {0.0, 0.0}, 0.0, 0.0,0.0},     //  9. parameter delta to zero -> no reaction
        DriverVel{10.0, 10.0, 4.0, {inf, inf}, {0.0, 0.0}, 0.0, 0.0, 0.0},     // 10. velocities equal -> no reaction
        DriverVel{100.0, 100.0, 4.0, {inf, inf}, {0.0, 0.0}, 0.0, 0.0, 0.0},     // 11. velocities equal -> no reaction
        DriverVel{100.0, 120.0, 4.0, {inf, inf}, {0.0, 0.0}, 0.0, 0.0, 0.5177},  // 12. accelerate to reach desired velocity
        DriverVel{10.0, 20.0, 4.0, {inf, inf}, {0.0, 0.0}, 0.0, 0.0, 0.9375},  // 13. accelerate to reach desired velocity
        DriverVel{10.0, 30.0, 4.0, {inf, inf}, {0.0, 0.0}, 0.0, 0.0, 0.9877},  // 14. accelerate to reach desired velocity
        DriverVel{20.0, 30.0, 4.0, {inf, inf}, {0.0, 0.0}, 0.0, 0.0, 0.8025},  // 15. accelerate to reach desired velocity
        DriverVel{20.0, 10.0, 4.0, {inf, inf}, {0.0, 0.0}, 0.0, 0.0, -1.0},     // 16. should be symmetric to 6.
        DriverVel{40.0, 30.0, 4.0, {inf, inf}, {0.0, 0.0}, 0.0, 0.0, -0.8025},  // 17. should be symmetric to 14.
        DriverVel{140.0, 120.0, 4.0, {inf, inf}, {0.0, 0.0}, 0.0, 0.0, -0.5177},  // 18. should be symmetric to 12.

        //         v      vDes  de1  ds              vs            de2  T     r
        DriverVel{20.0, 10.0, 4.0, {500.0, 500.0}, {1.0, 1.0}, 2.0, 10.0, -1.0}, // 19. Prediction test (too far away)
        DriverVel{20.0, 20.0, 4.0, {inf, 0.0}, {10.0, 10.0}, 2.0, 10.0, -1.0}, // 19. Prediction test (distance zero)
        DriverVel{20.0, 20.0, 4.0, {inf, -1.0}, {10.0, 10.0}, 2.0, 10.0, -1.0}  // 19. Prediction test (distance zero)

));


TEST(ModelTestVelocity, FailureTest) {

    EXPECT_THROW(IDMSpeedReaction(inf, 10.0, 4.0), std::invalid_argument);
    EXPECT_THROW(IDMSpeedReaction(-1e-9, 10.0, 4.0), std::invalid_argument);
    EXPECT_THROW(IDMSpeedReaction(1.0, -1e9, 4.0), std::invalid_argument);

}

#pragma clang diagnostic pop