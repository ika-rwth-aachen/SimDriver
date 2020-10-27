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
// Created by Jens Klimke on 2020-03-05.
// Contributors:
//
// MC_SalvucciAndGray.cpp

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma ide diagnostic ignored "cert-err58-cpp"

#include <gtest/gtest.h>
#include <SimDriver/model_collection.h>
#include <cmath>

using namespace sim_driver::models;

const static double inf = INFINITY;

#define DEG45 (M_PI * 0.25)
#define DEG90 (M_PI * 0.5)

struct DriverSalvucci {

    // ego state
    double x;
    double y;
    double v;
    double dy;

    // results
    double theta;
    double dTheta;
    double result;

};


struct ModelTestSalvucci : public testing::Test, public testing::WithParamInterface<DriverSalvucci> {

    ModelTestSalvucci() = default;

};


TEST_P(ModelTestSalvucci, IDMFreeTest) {

    // declare results
    double theta;
    double dTheta;

    // calculate
    auto p = GetParam();
    double result = SalvucciAndGray(p.x, p.y, p.v, p.dy, 2.0, 0.5, theta, dTheta);

    // check acceleration
    EXPECT_NEAR(p.result, result, 1e-3);
    EXPECT_NEAR(p.theta, theta, 1e-3);
    EXPECT_NEAR(p.dTheta, dTheta, 1e-3);

}


INSTANTIATE_TEST_CASE_P(Free, ModelTestSalvucci, testing::Values(

        //                 x     y    dx    dy      th     dth     res
        DriverSalvucci{0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0}, //  0. x and y are zero (special case)
        DriverSalvucci{10.0, 10.0, 0.0, 0.0, DEG45, 0.0, DEG90}, //  1. x and y in 45 degree, no dynamics
        DriverSalvucci{10.0, 1.0, 0.0, 0.0, 0.099, 0.0, 0.199}, //  2. x = 10, y = 1, no dynamics
        DriverSalvucci{10.0, -1.0, 0.0, 0.0, -0.099, 0.0, -0.199}, //  3. same as 2 but y negative
        DriverSalvucci{-10.0, -1.0, 0.0, 0.0, -3.042, 0.0, -6.083}, //  4. same as 2 but x negative
        DriverSalvucci{10.0, 1.0, 1.0, 0.1, 0.099, 0.019, 0.209}  //  5. with dynamics

));


TEST(ModelTestSalvucci, FailureTest) {

    EXPECT_THROW(IDMSpeedReaction(inf, 10.0, 4.0), std::invalid_argument);
    EXPECT_THROW(IDMSpeedReaction(-1e-9, 10.0, 4.0), std::invalid_argument);
    EXPECT_THROW(IDMSpeedReaction(1.0, -1e9, 4.0), std::invalid_argument);

}

#pragma clang diagnostic pop