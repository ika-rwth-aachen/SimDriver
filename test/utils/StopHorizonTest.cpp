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
#include <SimDriver/utils/StopHorizon.h>


class StopHorizonTest : public ::testing::Test, public sim_driver::StopHorizon {

};


TEST_F(StopHorizonTest, Update) {

    // initialize
    init(10.1);

    EXPECT_DOUBLE_EQ(10.1, _sActual);
    EXPECT_EQ(0, _elements.size());


    // update
    update(10.5, 0.1);

    EXPECT_DOUBLE_EQ(10.5, _sActual);
    EXPECT_EQ(0, _elements.size());


}


TEST_F(StopHorizonTest, AddElements) {

    // initialize
    init(10.1);

    // add element
    EXPECT_TRUE( addStopPoint(1, 30.0, 1.0));
    EXPECT_FALSE(addStopPoint(1, 40.0, 0.5));
    EXPECT_TRUE( addStopPoint(2, 60.0, 1.5));
    EXPECT_TRUE( addStopPoint(3, 90.0, 1.0));

    // update
    update(12.1, 0.1);

    // add elements
    EXPECT_TRUE( addStopPoint(4, 120.0, 0.5));
    EXPECT_TRUE( addStopPoint(5, 150.0, 1.0));

    // add element which is far behind
    EXPECT_FALSE(addStopPoint(6,   0.0, 1.3));

    // check
    ASSERT_EQ(5, _elements.size());

    EXPECT_DOUBLE_EQ(30.0, _elements.at(1).s);
    EXPECT_DOUBLE_EQ(10.1, _elements.at(1).sStart);
    EXPECT_DOUBLE_EQ( 1.0, _elements.at(1).standingTime);
    EXPECT_EQ(INFINITY,    _elements.at(1).timeStartStanding);
    EXPECT_EQ(false,       _elements.at(1).passed);

    EXPECT_DOUBLE_EQ(60.0, _elements.at(2).s);
    EXPECT_DOUBLE_EQ(10.1, _elements.at(2).sStart);
    EXPECT_DOUBLE_EQ( 1.5, _elements.at(2).standingTime);
    EXPECT_EQ(INFINITY,    _elements.at(2).timeStartStanding);
    EXPECT_EQ(false,       _elements.at(2).passed);

    EXPECT_DOUBLE_EQ(90.0, _elements.at(3).s);
    EXPECT_DOUBLE_EQ(10.1, _elements.at(3).sStart);
    EXPECT_DOUBLE_EQ( 1.0, _elements.at(3).standingTime);
    EXPECT_EQ(INFINITY,    _elements.at(3).timeStartStanding);
    EXPECT_EQ(false,       _elements.at(3).passed);

    EXPECT_DOUBLE_EQ(120.0, _elements.at(4).s);
    EXPECT_DOUBLE_EQ( 12.1, _elements.at(4).sStart);
    EXPECT_DOUBLE_EQ(  0.5, _elements.at(4).standingTime);
    EXPECT_EQ(INFINITY,     _elements.at(4).timeStartStanding);
    EXPECT_EQ(false,        _elements.at(4).passed);

    EXPECT_DOUBLE_EQ(150.0, _elements.at(5).s);
    EXPECT_DOUBLE_EQ( 12.1, _elements.at(5).sStart);
    EXPECT_DOUBLE_EQ(  1.0, _elements.at(5).standingTime);
    EXPECT_EQ(INFINITY,     _elements.at(5).timeStartStanding);
    EXPECT_EQ(false,        _elements.at(5).passed);

}


TEST_F(StopHorizonTest, UpdateElements) {

    // initialize
    init(10.1);

    // add element
    addStopPoint(1, 30.0, 1.0);
    addStopPoint(2, 60.0, 1.5);
    addStopPoint(3, 90.0, 1.0);

    // update
    update(30.0, 0.1);

    // add elements
    addStopPoint(4, 120.0, 0.5);
    addStopPoint(5, 150.0, 1.0);

    // marked as stopped
    EXPECT_TRUE(stopped(1, 0.1));

    // check
    EXPECT_DOUBLE_EQ(30.0, _elements.at(1).s);
    EXPECT_DOUBLE_EQ(10.1, _elements.at(1).sStart);
    EXPECT_DOUBLE_EQ( 1.0, _elements.at(1).standingTime);
    EXPECT_DOUBLE_EQ( 0.1, _elements.at(1).timeStartStanding);
    EXPECT_EQ(false,       _elements.at(1).passed);


    // update & mark again
    update(30.0, 0.2);
    EXPECT_FALSE(stopped(1, 0.2));

    // check
    EXPECT_DOUBLE_EQ(30.0, _elements.at(1).s);
    EXPECT_DOUBLE_EQ(10.1, _elements.at(1).sStart);
    EXPECT_DOUBLE_EQ( 1.0, _elements.at(1).standingTime);
    EXPECT_DOUBLE_EQ( 0.1, _elements.at(1).timeStartStanding); // not set because it was already set
    EXPECT_EQ(false,       _elements.at(1).passed);


    // update
    update(30.0, 1.1);

    // check
    EXPECT_DOUBLE_EQ(30.0, _elements.at(1).s);
    EXPECT_DOUBLE_EQ(10.1, _elements.at(1).sStart);
    EXPECT_DOUBLE_EQ( 1.0, _elements.at(1).standingTime);
    EXPECT_DOUBLE_EQ( 0.1, _elements.at(1).timeStartStanding);
    EXPECT_EQ(true,        _elements.at(1).passed);


    // update
    update(30.0, 1.20001);

    // check
    EXPECT_DOUBLE_EQ(30.0, _elements.at(1).s);
    EXPECT_DOUBLE_EQ(10.1, _elements.at(1).sStart);
    EXPECT_DOUBLE_EQ( 1.0, _elements.at(1).standingTime);
    EXPECT_DOUBLE_EQ( 0.1, _elements.at(1).timeStartStanding);
    EXPECT_EQ(true,        _elements.at(1).passed);

    // update
    update(40.0, 2.2);

    ASSERT_EQ(4, _elements.size());
    EXPECT_THROW(_elements.at(1), std::out_of_range);

}


TEST_F(StopHorizonTest, GetNextStop) {

    // initialize
    init(10.1);

    auto sp = getNextStop();
    EXPECT_LT(10000, sp.id);
    EXPECT_EQ(INFINITY, sp.ds);

    // add element
    addStopPoint(1, 30.0, 1.0);
    addStopPoint(2, 60.0, 1.5);
    addStopPoint(3, 90.0, 1.0);

    // update
    update(20.0, 0.1);

    // get stop point
    sp = getNextStop();
    EXPECT_EQ(1, sp.id);
    EXPECT_NEAR(10.0, sp.ds, 1e-9);

    // update
    update(30.1, 0.1);

    // get stop point
    sp = getNextStop();
    EXPECT_EQ(1, sp.id);
    EXPECT_NEAR(-0.1, sp.ds, 1e-9);

    // add elements
    addStopPoint(4, 120.0, 0.5);
    addStopPoint(5, 150.0, 1.0);

    // marked as stopped
    stopped(1, 0.2);
    stopped(2, 0.2);

    // update
    update(32.0, 1.2);

    // get stop point
    sp = getNextStop();
    EXPECT_EQ(2, sp.id);
    EXPECT_NEAR(28.0, sp.ds, 1e-9);

    // update
    update(40.0, 2.0);

    // get stop point
    sp = getNextStop();
    EXPECT_EQ(3, sp.id);
    EXPECT_NEAR(50.0, sp.ds, 1e-9);

}


#pragma clang diagnostic pop
