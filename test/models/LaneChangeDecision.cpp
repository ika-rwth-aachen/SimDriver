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
// DM_LaneChangeDecision.cpp


#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma ide diagnostic ignored "cert-err58-cpp"

#include <gtest/gtest.h>
#include <cmath>
#include <SimDriver/model_collection.h>

using namespace sim_driver::models;

const static double inf = INFINITY;

struct DriverLC {

    struct Object {
        double v;
        double ds;
    };

    // ego state
    double v;
    double vDes;

    // traffic situation
    Object frontEgo;
    Object rearEgo;
    Object frontTarget;
    Object rearTarget;

    // parameters
    double bSafe;
    double aThreshold;
    double politeness;

    // results
    double safety;
    double incentive;

};

typedef DriverLC::Object O;

struct ModelTestLaneChange : public testing::Test, public testing::WithParamInterface<DriverLC> {

    ModelTestLaneChange() = default;

};


TEST_P(ModelTestLaneChange, LaneChangeDecision) {

    // parameters
    auto p = GetParam();

    // results
    double safety;
    double incentive;

    // execute model
    MOBILOriginal(
            safety, incentive, p.v, p.vDes, 1.5, 2.0, 1.0, 1.5,
            p.frontEgo.ds, p.frontEgo.v,
            p.frontTarget.ds, p.frontTarget.v,
            p.rearEgo.ds, p.rearEgo.v,
            p.rearTarget.ds, p.rearTarget.v,
            p.bSafe, p.aThreshold, p.politeness);

    // check acceleration
    EXPECT_NEAR(safety, p.safety, 1e-4);
    EXPECT_NEAR(incentive, p.incentive, 1e-4);

}


INSTANTIATE_TEST_CASE_P(LC, ModelTestLaneChange, testing::Values(

        //       v     vDes  front-ego    rear-ego      front-tar    rear-tar      bSaf  aThr pol  saf   inc
        DriverLC{10.0, 20.0, O{0.0, inf}, O{0.0, -inf}, O{0.0, inf}, O{0.0, -inf}, -1.5, 0.5, 1.0, 0.375, -1.0}
        // TODO: implement more tests

));


TEST(ModelTestLaneChange, FailureTest) {

}


TEST(ModelTestLaneChange, IDMTest) {

    EXPECT_DOUBLE_EQ(0.0, IDMOriginal(0.0, 0.0, 100.0, 1.0, 1.8, 2.0, 1.0, 1.5));

}


#pragma clang diagnostic pop