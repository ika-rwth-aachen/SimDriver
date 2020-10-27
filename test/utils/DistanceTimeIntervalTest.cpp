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
// Created by Jens Klimke on 2020-04-03.
// Contributors:
//
// InternalHorizonTest.h

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma ide diagnostic ignored "cert-err58-cpp"

#include <gtest/gtest.h>
#include <SimDriver/utils/DistanceTimeInterval.h>


class DistanceTimeIntervalTest : public ::testing::Test, public sim_driver::DistanceTimeInterval {

};



TEST_F(DistanceTimeIntervalTest, Initialization) {

    EXPECT_EQ(0.0, _actualPosition);
    EXPECT_EQ(0.0, _actualTime);
    EXPECT_EQ(1.0, _scale);
    EXPECT_FALSE(isSet());

    reset();
    update(1.0, 2.0);

    EXPECT_FALSE(isSet());

    EXPECT_EQ(1.0, _actualPosition);
    EXPECT_EQ(2.0, _actualTime);

    EXPECT_EQ(INFINITY, _startPosition);
    EXPECT_EQ(INFINITY, _startTime);
    EXPECT_EQ(INFINITY, _endPosition);
    EXPECT_EQ(INFINITY, _endTime);


    update(3.0, 4.0);

    EXPECT_EQ(3.0, _actualPosition);
    EXPECT_EQ(4.0, _actualTime);

    EXPECT_EQ(INFINITY, _startPosition);
    EXPECT_EQ(INFINITY, _startTime);
    EXPECT_EQ(INFINITY, _endPosition);
    EXPECT_EQ(INFINITY, _endTime);

}


TEST_F(DistanceTimeIntervalTest, SetInterval) {

    reset();
    update(1.0, 2.0);
    setTimeInterval(10.0);

    EXPECT_EQ(INFINITY, _startPosition);
    EXPECT_EQ(INFINITY, _endPosition);
    EXPECT_EQ(2.0, _startTime);
    EXPECT_EQ(12.0, _endTime);


    update(3.0, 4.0);

    EXPECT_EQ(INFINITY, _startPosition);
    EXPECT_EQ(INFINITY, _endPosition);
    EXPECT_EQ(2.0, _startTime);
    EXPECT_EQ(12.0, _endTime);


    setEndPosition(20.0);

    EXPECT_EQ(3.0, _startPosition);
    EXPECT_EQ(20.0, _endPosition);
    EXPECT_EQ(2.0, _startTime);
    EXPECT_EQ(12.0, _endTime);


    reset();

    EXPECT_EQ(INFINITY, _startPosition);
    EXPECT_EQ(INFINITY, _startTime);
    EXPECT_EQ(INFINITY, _endPosition);
    EXPECT_EQ(INFINITY, _endTime);

}



TEST_F(DistanceTimeIntervalTest, GetValueByPosition) {

    reset();
    update(1.0, 2.0);
    setEndPosition(11.0);

    EXPECT_TRUE(isSet());
    EXPECT_NEAR(0.0, getScaledFactor(), 1e-6);

    update(6.0, 7.0);
    EXPECT_NEAR(0.5, getScaledFactor(), 1e-6);

    update(11.0, 12.0);
    EXPECT_NEAR(1.0, getScaledFactor(), 1e-6);

    update(12.0, 14.0);
    EXPECT_NEAR(1.0, getScaledFactor(), 1e-6);

}



TEST_F(DistanceTimeIntervalTest, GetValueByTime) {

    reset();
    update(1.0, 2.0);
    setTimeInterval(10.0);

    EXPECT_TRUE(isSet());
    EXPECT_NEAR(0.0, getScaledFactor(), 1e-6);

    update(6.0, 7.0);
    EXPECT_NEAR(0.5, getScaledFactor(), 1e-6);

    update(11.0, 12.0);
    EXPECT_NEAR(1.0, getScaledFactor(), 1e-6);

    update(12.0, 13.0);
    EXPECT_NEAR(1.0, getScaledFactor(), 1e-6);

}


TEST_F(DistanceTimeIntervalTest, GetValueByDistanceAndTime) {

    reset();
    update(0.0, 2.0);

    // set scale
    setScale(2.0);

    // check
    EXPECT_DOUBLE_EQ(1.0, _delta);
    EXPECT_DOUBLE_EQ(2.0, _scale);
    EXPECT_DOUBLE_EQ(2.0, getScale());

    // set intervals
    setEndPosition(20.0);
    setTimeInterval(10.0);

    EXPECT_TRUE(isSet());
    EXPECT_NEAR(0.0, getScaledFactor(), 1e-6);

    update(10.0, 5.0);
    EXPECT_NEAR(1.0, getScaledFactor(), 1e-6);

    update(10.0, 6.0);
    EXPECT_NEAR(1.0, getScaledFactor(), 1e-6);

    update(18.0, 12.0);
    EXPECT_NEAR(2.0, getScaledFactor(), 1e-6);

    update(20.0, 12.0);
    EXPECT_NEAR(2.0, getScaledFactor(), 1e-6);

    update(12.0, 14.0);
    EXPECT_NEAR(2.0, getScaledFactor(), 1e-6);

}


TEST_F(DistanceTimeIntervalTest, GetValueByDistanceAndTimeWithDelta) {

    reset();
    setDelta(2.0);
    update(0.0, 2.0);

    // set scale
    setScale(2.0);

    // check
    EXPECT_DOUBLE_EQ(2.0, _delta);
    EXPECT_DOUBLE_EQ(2.0, _scale);
    EXPECT_DOUBLE_EQ(2.0, getScale());

    // check not set
    EXPECT_FALSE(isSet());
    EXPECT_NEAR(0.0, getScaledFactor(), 1e-6);

    // set intervals
    setEndPosition(20.0);
    setTimeInterval(10.0);

    EXPECT_TRUE(isSet());
    EXPECT_NEAR(0.0, getScaledFactor(), 1e-6);

    update(10.0, 5.0);
    EXPECT_NEAR(0.5, getScaledFactor(), 1e-6);

    update(10.0, 6.0);
    EXPECT_NEAR(0.5, getScaledFactor(), 1e-6);

    update(18.0, 12.0);
    EXPECT_NEAR(2.0, getScaledFactor(), 1e-6);

    update(20.0, 12.0);
    EXPECT_NEAR(2.0, getScaledFactor(), 1e-6);

    update(12.0, 14.0);
    EXPECT_NEAR(2.0, getScaledFactor(), 1e-6);

}


#pragma clang diagnostic pop
