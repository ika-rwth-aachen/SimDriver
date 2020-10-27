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
#include <SimDriver/utils/VelocityHorizon.h>


class VelocityHorizonTest : public ::testing::Test, public sim_driver::VelocityHorizon {

};



TEST_F(VelocityHorizonTest, Initialization) {

    // init
    init(0.0, 11);
    for (size_t i = 0; i < _elements.size(); ++i)
        EXPECT_DOUBLE_EQ((double) i, _elements.at(i).s);

    // init again
    init(10.1, 11);

    // check attributes
    EXPECT_DOUBLE_EQ(10.0, _offset);
    EXPECT_EQ(11, _elements.size());

    // check points
    for (size_t i = 0; i < _elements.size(); ++i) {

        // index and position
        EXPECT_EQ(i, _elements.at(i).i);
        EXPECT_DOUBLE_EQ((double) i + 10.0, _elements.at(i).s);
        EXPECT_DOUBLE_EQ((double) i +  9.0, _elements.at(i).sCont);

        // unset values
        EXPECT_EQ(INFINITY, _elements.at(i).ds);
        EXPECT_EQ(INFINITY, _elements.at(i).vCont);
        EXPECT_EQ(INFINITY, _elements.at(i).vRule);

    }

}



TEST_F(VelocityHorizonTest, Access) {

    // init
    init(10.1, 11);

    // check front and back
    EXPECT_DOUBLE_EQ(10.0, _elements.front().s);
    EXPECT_DOUBLE_EQ(20.0, _elements.back().s);

    // get index
    EXPECT_EQ( 0, getIndexBefore(10.0));
    EXPECT_EQ( 0, getIndexBefore(10.5));
    EXPECT_EQ( 1, getIndexBefore(11.0));
    EXPECT_EQ( 1, getIndexBefore(11.5));
    EXPECT_EQ(10, getIndexBefore(20.0));

    // move a step
    update(15.1);

    // check front and back
    EXPECT_DOUBLE_EQ(15.0, _elements.front().s);
    EXPECT_DOUBLE_EQ(25.0, _elements.back().s);

    // get index before
    EXPECT_EQ( 0, getIndexBefore(15.0));
    EXPECT_EQ( 0, getIndexBefore(15.5));
    EXPECT_EQ( 1, getIndexBefore(16.0));
    EXPECT_EQ( 1, getIndexBefore(16.5));
    EXPECT_EQ(10, getIndexBefore(25.0));

    // get index after
    EXPECT_EQ( 0, getIndexAfter(15.0));
    EXPECT_EQ( 1, getIndexAfter(15.5));
    EXPECT_EQ( 1, getIndexAfter(16.0));
    EXPECT_EQ( 2, getIndexAfter(16.5));
    EXPECT_EQ(10, getIndexAfter(25.0));

}



TEST_F(VelocityHorizonTest, SetRuleSpeed) {

    // init
    init(10.1, 101);

    // sets the speed
    setMaxVelocity(20.0);
    EXPECT_DOUBLE_EQ(20.0, _vMax);


    // set rule speed
    updateSpeedRuleInInterval( 5.0,  15.0, 10.0);
    updateSpeedRuleInInterval(50.1,  59.9, 11.0);
    updateSpeedRuleInInterval(45.0,  55.0, 12.0);
    updateSpeedRuleInInterval(90.0, 120.0, 13.0);

    // check elements
    for(auto &e : _elements) {

        if(e.s >= 10.0 && e.s <= 15.0)
            EXPECT_DOUBLE_EQ(10.0, e.vRule);
        else if(e.s >= 45.0 && e.s < 50.0)
            EXPECT_DOUBLE_EQ(12.0, e.vRule);
        else if(e.s >= 50.0 && e.s <= 60.0)
            EXPECT_DOUBLE_EQ(11.0, e.vRule);
        else if(e.s >= 90.0 && e.s <= 110.0)
            EXPECT_DOUBLE_EQ(13.0, e.vRule);
        else
            EXPECT_EQ(INFINITY, e.vRule);

    }


    // do a step
    update(15.1);

    // check elements
    for(auto &e : _elements) {

        if(e.s >= 10.0 && e.s <= 15.0)
            EXPECT_DOUBLE_EQ(10.0, e.vRule);
        else if(e.s >= 45.0 && e.s < 50.0)
            EXPECT_DOUBLE_EQ(12.0, e.vRule);
        else if(e.s >= 50.0 && e.s <= 60.0)
            EXPECT_DOUBLE_EQ(11.0, e.vRule);
        else if(e.s >= 90.0 && e.s <= 110.0)
            EXPECT_DOUBLE_EQ(13.0, e.vRule);
        else
            EXPECT_EQ(INFINITY, e.vRule);

    }


    // reset
    resetSpeedRule();
    for(auto &e : _elements)
        EXPECT_EQ(INFINITY, e.vRule);

    // set rule speed
    updateSpeedRuleInInterval( 5.0,  15.0, 10.0);
    updateSpeedRuleInInterval(50.1,  59.9, 11.0);
    updateSpeedRuleInInterval(45.0,  55.0, 12.0);
    updateSpeedRuleInInterval(90.0, 120.0, 13.0);

    // check elements
    for(auto &e : _elements) {

        if(e.s >= 15.0 && e.s <= 15.0)
            EXPECT_DOUBLE_EQ(10.0, e.vRule);
        else if(e.s >= 45.0 && e.s < 50.0)
            EXPECT_DOUBLE_EQ(12.0, e.vRule);
        else if(e.s >= 50.0 && e.s <= 60.0)
            EXPECT_DOUBLE_EQ(11.0, e.vRule);
        else if(e.s >= 90.0 && e.s <= 120.0)
            EXPECT_DOUBLE_EQ(13.0, e.vRule);
        else
            EXPECT_EQ(INFINITY, e.vRule);

    }

}



TEST_F(VelocityHorizonTest, ContinuousUpdate) {

    // init
    init(10.1, 101);

    // update continuous point
    updateContinuousPoint(15.1, 5.0);

    EXPECT_DOUBLE_EQ( 5.0, _elements.at(6).vCont);
    EXPECT_DOUBLE_EQ(15.1, _elements.at(6).sCont);

    EXPECT_EQ(INFINITY, _elements.at(5).vCont);
    EXPECT_DOUBLE_EQ(14.0, _elements.at(5).sCont);

    EXPECT_EQ(INFINITY, _elements.at(7).vCont);
    EXPECT_DOUBLE_EQ(16.0, _elements.at(7).sCont);


    // update continuous point
    updateContinuousPoint(15.8, 4.0);

    EXPECT_DOUBLE_EQ( 4.0, _elements.at(6).vCont);
    EXPECT_DOUBLE_EQ(15.8, _elements.at(6).sCont);

    EXPECT_EQ(INFINITY, _elements.at(5).vCont);
    EXPECT_DOUBLE_EQ(14.0, _elements.at(5).sCont);

    EXPECT_EQ(INFINITY, _elements.at(7).vCont);
    EXPECT_DOUBLE_EQ(16.0, _elements.at(7).sCont);


    // update continuous point
    updateContinuousPoint(16.0, 4.0);

    EXPECT_DOUBLE_EQ( 4.0, _elements.at(6).vCont);
    EXPECT_DOUBLE_EQ(16.0, _elements.at(6).sCont);

    EXPECT_EQ(INFINITY, _elements.at(5).vCont);
    EXPECT_DOUBLE_EQ(14.0, _elements.at(5).sCont);

    EXPECT_EQ(INFINITY, _elements.at(7).vCont);
    EXPECT_DOUBLE_EQ(16.0, _elements.at(7).sCont);


    // update continuous point
    updateContinuousPoint(16.1, 4.0);

    EXPECT_DOUBLE_EQ( 4.0, _elements.at(6).vCont);
    EXPECT_DOUBLE_EQ(16.0, _elements.at(6).sCont);

    EXPECT_EQ(INFINITY, _elements.at(5).vCont);
    EXPECT_DOUBLE_EQ(14.0, _elements.at(5).sCont);

    EXPECT_DOUBLE_EQ( 4.0, _elements.at(7).vCont);
    EXPECT_DOUBLE_EQ(16.1, _elements.at(7).sCont);


}



TEST_F(VelocityHorizonTest, Mean) {

    // init
    init(10.1, 101);

    // set maximum speed
    setMaxVelocity(20.0);

    // set rule speed
    updateSpeedRuleInInterval( 5.0,  20.0, 10.0);
    updateSpeedRuleInInterval(50.1,  59.9, 11.0);
    updateSpeedRuleInInterval(45.0,  55.0, 12.0);
    updateSpeedRuleInInterval(90.0, 120.0, 13.0);

    // update continuous point
    updateContinuousPoint(15.0, 7.0);
    updateContinuousPoint(16.1, 7.0);
    updateContinuousPoint(17.2, 7.0);
    updateContinuousPoint(18.3, 7.0);
    updateContinuousPoint(19.4, 7.0);
    updateContinuousPoint(20.5, 7.0);
    updateContinuousPoint(21.6, 7.0);
    updateContinuousPoint(22.7, 7.0);
    updateContinuousPoint(23.8, 7.0);
    updateContinuousPoint(24.9, 7.0);
    updateContinuousPoint(26.0, 7.0);

    // calculate mean
    EXPECT_NEAR(7.120, mean(10.1, 110.1, 0.0), 0.001);
    EXPECT_NEAR(7.000, mean(10.1, 110.1, 1.0), 0.001);
    EXPECT_NEAR(7.000, mean(10.1, 110.1, 2.0), 0.001);

}


#pragma clang diagnostic pop
