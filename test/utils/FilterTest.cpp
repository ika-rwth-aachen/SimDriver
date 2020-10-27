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
// Created by Jens Klimke on 2020-04-04.
// Contributors:
//
// FilterTest.h

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma ide diagnostic ignored "cert-err58-cpp"

#include <gtest/gtest.h>
#include <SimDriver/utils/Filter.h>


class FilerTest : public ::testing::Test, public sim_driver::Filter {
};

TEST_F(FilerTest, Creation) {

    // init
    this->init(6);
    EXPECT_EQ(6, this->n);

    // add values
    EXPECT_NEAR(0.0, this->value(0.0),  1e-9);
    EXPECT_NEAR(0.5, this->value(1.0),  1e-9);
    EXPECT_NEAR(1.0, this->value(2.0),  1e-9);
    EXPECT_NEAR(1.5, this->value(3.0),  1e-9);
    EXPECT_NEAR(2.0, this->value(4.0),  1e-9);
    EXPECT_NEAR(2.5, this->value(5.0),  1e-9);
    EXPECT_NEAR(3.5, this->value(6.0),  1e-9);
    EXPECT_NEAR(4.5, this->value(7.0),  1e-9);
    EXPECT_NEAR(5.5, this->value(8.0),  1e-9);
    EXPECT_NEAR(6.5, this->value(9.0),  1e-9);
    EXPECT_NEAR(7.5, this->value(10.0), 1e-9);
    EXPECT_NEAR(8.5, this->value(11.0), 1e-9);

    this->init(6);
    EXPECT_DOUBLE_EQ(0.0, this->value());

}

#pragma clang diagnostic pop
