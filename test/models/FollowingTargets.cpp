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
// MC_FollowingTargets.cpp

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma ide diagnostic ignored "cert-err58-cpp"

#include <gtest/gtest.h>
#include <cmath>
#include <SimDriver/model_collection.h>

using namespace sim_driver::models;

const static double inf = INFINITY;

struct DriverFol {

    double ds;
    double v;
    double vTar;
    double T;
    double s0;
    double reaction;

};


struct ModelTestFol : public testing::Test, public testing::WithParamInterface<DriverFol> {

    ModelTestFol() = default;

};


TEST_P(ModelTestFol, FollowTest) {

    auto as = GetParam();
    double a = IDMFollowReaction(as.ds, as.vTar, as.v, as.T, as.s0, 1, -1.5);

    // check acceleration
    EXPECT_NEAR(as.reaction, a, 1e-3);

}


INSTANTIATE_TEST_CASE_P(Follow, ModelTestFol, testing::Values(

        //        ds   v     vT   T    s0   r
        DriverFol{inf, 10.0, 0.0, 1.0, 0.0, 0.0},  //  0. no target in range, should result in no reaction
        DriverFol{inf, 10.0, 0.0, 0.0, 0.0, 0.0},  //  1. no target in range (dsStar = 0), shouldn't result in nan
        DriverFol{inf, 10.0, 0.0, inf, inf, 0.0},  //  2. all values to inf, should result in no reaction
        DriverFol{0.0, 0.0, 0.0, 0.0, 0.0, 1.0},  //  3. all values to zero, should result in full reaction
        DriverFol{0.0, 10.0, 10.0, 0.0, 0.0, 1.0},  //  4. distance is reached (ds = 0), same velocity
        DriverFol{2.0, 10.0, 10.0, 0.0, 2.0, 1.0},  //  5. distance is reached (ds = 2, s0 = 2), same velocity
        DriverFol{10.0, 10.0, 10.0, 1.0, 0.0, 1.0},  //  6. distance is reached, same velocity
        DriverFol{12.0, 10.0, 10.0, 1.0, 2.0, 1.0},  //  7. distance is reached, same velocity
        DriverFol{20.0, 20.0, 20.0, 1.0, 0.0, 1.0},  //  8. distance is reached, same velocity
        DriverFol{2.0, 0.0, 0.0, 0.0, 2.0, 1.0},  //  9. distance is reached, standing
        DriverFol{2.0, 0.0, 2.0, 1.0, 0.0, 0.0},  // 10. distance is reached, standing
        DriverFol{2.0, 0.0, 0.0, 1.0, 2.0, 1.0},  // 11. distance is reached, standing
        DriverFol{100.0, 10.0, 10.0, 1.0, 2.0, 0.014},  // 12. reaction in 100 m distance, same speed
        DriverFol{500.0, 20.0, 10.0, 1.0, 2.0, 0.042},  // 13. reaction in 500 m distance, higher speed
        DriverFol{500.0, 50.0, 10.0, 1.0, 2.0, 3.017},  // 14. reaction in 499 m distance, higher speed
        DriverFol{10.0, 10.0, 10.0, 1.0, 2.0, 1.440}   // 15. reaction in 499 m distance, higher speed

));


TEST(ModelTestFol, FailureTest) {

    EXPECT_THROW(IDMFollowReaction(500.0, -1e-9, 20.0, 1.0, 2.0, 1, -1.5), std::invalid_argument);
    EXPECT_THROW(IDMFollowReaction(500.0, 10.0, -1e-9, 1.0, 2.0, 1, -1.5), std::invalid_argument);
    EXPECT_THROW(IDMFollowReaction(500.0, 10.0, inf, 1.0, 2.0, 1, -1.5), std::invalid_argument);

    EXPECT_EQ(inf, IDMFollowReaction(-10.0, 10.0, 10.0, 1.0, 2.0, 1, -1.5));

}

#pragma clang diagnostic pop