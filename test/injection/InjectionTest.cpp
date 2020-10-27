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
// Created by Jens Klimke on 2020-02-18.
// Contributors:
//
// InjectionTest.h

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma ide diagnostic ignored "cert-err58-cpp"

#include <gtest/gtest.h>
#include <Injection/Injection.h>


TEST(InjectionTest, Injection) {

    using namespace sim_driver::injection;

    int owner1 = 1;
    int owner2 = 1;

    double d1 = 1.0;
    double d2 = 2.0;
    double d3 = 3.0;

    Injection<double> i1{};
    i1.registerValue(&d1, &owner1);
    EXPECT_EQ(&d1, i1.ptr());

    Injection<double> i2{};
    i2.registerValue(&d2, &owner1);
    EXPECT_EQ(&d2, i2.ptr());

    Injection<double> i3{};
    i3.registerValue(&d3, &owner2);
    EXPECT_EQ(&d3, i3.ptr());

    EXPECT_EQ(2, Interface::_index.size());
    EXPECT_NE(Interface::_index.end(), Interface::_index.find(&owner1));

    EXPECT_EQ(2, Interface::_index[&owner1].size());
    EXPECT_EQ(&i1, Interface::_index[&owner1].front());
    EXPECT_EQ(&i2, Interface::_index[&owner1].back());

    EXPECT_EQ(1, Interface::_index[&owner2].size());
    EXPECT_EQ(&i3, Interface::_index[&owner2].front());

    i1 = 10.0;
    i3 = 30.0;

    // not applied
    EXPECT_DOUBLE_EQ(1.0, (double) i1);
    EXPECT_DOUBLE_EQ(2.0, (double) i2);
    EXPECT_DOUBLE_EQ(3.0, (double) i3);

    // apply owner 1
    Interface::applyAll(&owner1);
    EXPECT_DOUBLE_EQ(10.0, (double) i1);
    EXPECT_DOUBLE_EQ(2.0, (double) i2);
    EXPECT_DOUBLE_EQ(3.0, (double) i3);

    // original values
    EXPECT_DOUBLE_EQ(10.0, d1);
    EXPECT_DOUBLE_EQ(2.0, d2);
    EXPECT_DOUBLE_EQ(3.0, d3);

    i1 = 100.0;
    i2 = 200.0;
    i3 = 300.0;

    // apply owner 2
    Interface::applyAll(&owner2);
    EXPECT_DOUBLE_EQ(10.0, (double) i1);
    EXPECT_DOUBLE_EQ(2.0, (double) i2);
    EXPECT_DOUBLE_EQ(300.0, (double) i3);

    // reset
    Interface::resetAll(&owner1);

    d1 = 1000.0;
    d2 = 2000.0;
    d3 = 3000.0;

    Interface::applyAll(&owner1);
    Interface::applyAll(&owner2);
    EXPECT_DOUBLE_EQ(1000.0, (double) i1);
    EXPECT_DOUBLE_EQ(2000.0, (double) i2);
    EXPECT_DOUBLE_EQ(300.0, (double) i3);

    Interface::resetAll(&owner2);
    EXPECT_DOUBLE_EQ(300.0, (double) i3);
    EXPECT_EQ(nullptr, i3._inj.get());

    // remove
    Interface::remove(&owner2);
    EXPECT_ANY_THROW(Interface::applyAll(&owner2));

}

#pragma clang diagnostic pop
