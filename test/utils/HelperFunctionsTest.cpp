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
// HelperTest.h

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma ide diagnostic ignored "cert-err58-cpp"

#include <gtest/gtest.h>
#include <SimDriver/utils/Math.h>

static const double inf = INFINITY;

TEST(HelperFunctionsTest, Interpolation) {

    using namespace sim_driver::math;

    // normal interpolation
    std::vector<double> x{};
    std::vector<double> y{};

    x = {0.0, 1.0, 2.0};
    y = {1.0, 3.0, 2.0};

    double eps = 1e-18;

    // hit exactly the points
    EXPECT_DOUBLE_EQ(1.0, interpolate(0.0, x.data(), y.data(), x.size(), 0));
    EXPECT_DOUBLE_EQ(3.0, interpolate(1.0, x.data(), y.data(), x.size(), 0));
    EXPECT_DOUBLE_EQ(2.0, interpolate(2.0, x.data(), y.data(), x.size(), 0));
    EXPECT_DOUBLE_EQ(1.0, interpolate(0.0 - eps, x.data(), y.data(), x.size(), 0));
    EXPECT_DOUBLE_EQ(2.0, interpolate(2.0 + eps, x.data(), y.data(), x.size(), 0));

    // check interpolation
    EXPECT_DOUBLE_EQ(1.2, interpolate(0.1, x.data(), y.data(), x.size(), 0));
    EXPECT_DOUBLE_EQ(2.0, interpolate(0.5, x.data(), y.data(), x.size(), 0));
    EXPECT_DOUBLE_EQ(2.8, interpolate(0.9, x.data(), y.data(), x.size(), 0));
    EXPECT_DOUBLE_EQ(2.9, interpolate(1.1, x.data(), y.data(), x.size(), 0));
    EXPECT_DOUBLE_EQ(2.5, interpolate(1.5, x.data(), y.data(), x.size(), 0));
    EXPECT_DOUBLE_EQ(2.1, interpolate(1.9, x.data(), y.data(), x.size(), 0));

    // check extrapolation
    EXPECT_DOUBLE_EQ(0.8, interpolate(-0.1, x.data(), y.data(), x.size(), 1));
    EXPECT_DOUBLE_EQ(1.9, interpolate(2.1, x.data(), y.data(), x.size(), 1));

    // check without extrapolation
    EXPECT_EQ(-inf, interpolate(-0.1, x.data(), y.data(), x.size(), 0));
    EXPECT_EQ(inf, interpolate(2.1, x.data(), y.data(), x.size(), 0));

    // check with first/last value
    EXPECT_EQ(1.0, interpolate(-0.1, x.data(), y.data(), x.size(), 2));
    EXPECT_EQ(2.0, interpolate(2.1, x.data(), y.data(), x.size(), 2));


    // special values
    x = {-inf, -inf, -inf, 0.0, 1.0, 2.0, inf, inf};
    y = {0.0, 0.0, 0.0, 1.0, 3.0, 2.0, 0.0, 0.0};

    // check interpolation
    EXPECT_DOUBLE_EQ(1.0, interpolate(0.0, x.data(), y.data(), x.size()));
    EXPECT_DOUBLE_EQ(-1.0, interpolate(-1.0, x.data(), y.data(), x.size()));
    EXPECT_DOUBLE_EQ(2.0, interpolate(0.5, x.data(), y.data(), x.size()));
    EXPECT_DOUBLE_EQ(2.0, interpolate(2.0, x.data(), y.data(), x.size()));
    EXPECT_DOUBLE_EQ(1.0, interpolate(3.0, x.data(), y.data(), x.size()));

    EXPECT_DOUBLE_EQ(1.0, interpolate(0.0, x.data(), y.data(), x.size(), 2));
    EXPECT_DOUBLE_EQ(1.0, interpolate(-1.0, x.data(), y.data(), x.size(), 2));
    EXPECT_DOUBLE_EQ(2.0, interpolate(0.5, x.data(), y.data(), x.size(), 2));
    EXPECT_DOUBLE_EQ(2.0, interpolate(2.0, x.data(), y.data(), x.size(), 2));
    EXPECT_DOUBLE_EQ(2.0, interpolate(3.0, x.data(), y.data(), x.size(), 2));


    // special values
    x = {-inf, -inf, inf, inf};
    y = {0.0, 0.0, 0.0, 0.0};

    // check
    EXPECT_THROW(interpolate(0.0, x.data(), y.data(), x.size()), std::invalid_argument);

}


TEST(HelperFunctionsTest, Scale) {

    using namespace sim_driver::math;

    EXPECT_DOUBLE_EQ(0.00000, scale(-0.1));
    EXPECT_DOUBLE_EQ(0.00000, scale( 0.0));
    EXPECT_DOUBLE_EQ(0.15625, scale( 0.25));
    EXPECT_DOUBLE_EQ(0.50000, scale( 0.5));
    EXPECT_DOUBLE_EQ(0.84375, scale( 0.75));
    EXPECT_DOUBLE_EQ(1.00000, scale( 1.0));
    EXPECT_DOUBLE_EQ(1.00000, scale( 1.1));

    EXPECT_DOUBLE_EQ(0.00, linScale(9.0, 20.0, 10.0));
    EXPECT_DOUBLE_EQ(0.00, linScale(10.0, 20.0, 10.0));
    EXPECT_DOUBLE_EQ(0.25, linScale(12.5, 20.0, 10.0));
    EXPECT_DOUBLE_EQ(0.50, linScale(15.0, 20.0, 10.0));
    EXPECT_DOUBLE_EQ(0.75, linScale(17.5, 20.0, 10.0));
    EXPECT_DOUBLE_EQ(1.00, linScale(20.0, 20.0, 10.0));
    EXPECT_DOUBLE_EQ(1.00, linScale(21.0, 20.0, 10.0));

    EXPECT_DOUBLE_EQ(0.00000, scale( 9.0, 20.0, 10.0, 0.0));
    EXPECT_DOUBLE_EQ(0.00000, scale(10.0, 20.0, 10.0, 0.0));
    EXPECT_DOUBLE_EQ(1.00000, scale(12.5, 20.0, 10.0, 0.0));
    EXPECT_DOUBLE_EQ(1.00000, scale(15.0, 20.0, 10.0, 0.0));
    EXPECT_DOUBLE_EQ(1.00000, scale(17.5, 20.0, 10.0, 0.0));
    EXPECT_DOUBLE_EQ(1.00000, scale(20.0, 20.0, 10.0, 0.0));
    EXPECT_DOUBLE_EQ(1.00000, scale(21.0, 20.0, 10.0, 0.0));

    EXPECT_DOUBLE_EQ(0.00000, scale( 9.0, 20.0, 10.0, 0.5));
    EXPECT_DOUBLE_EQ(0.00000, scale(10.0, 20.0, 10.0, 0.5));
    EXPECT_NEAR(     0.28808, scale(12.5, 20.0, 10.0, 0.5), 1e-5);
    EXPECT_DOUBLE_EQ(0.75000, scale(15.0, 20.0, 10.0, 0.5));
    EXPECT_NEAR(     0.97558, scale(17.5, 20.0, 10.0, 0.5), 1e-5);
    EXPECT_DOUBLE_EQ(1.00000, scale(20.0, 20.0, 10.0, 0.5));
    EXPECT_DOUBLE_EQ(1.00000, scale(21.0, 20.0, 10.0, 0.5));

    EXPECT_DOUBLE_EQ(0.00000, scale( 9.0, 20.0, 10.0, 1.0));
    EXPECT_DOUBLE_EQ(0.00000, scale(10.0, 20.0, 10.0, 1.0));
    EXPECT_DOUBLE_EQ(0.15625, scale(12.5, 20.0, 10.0, 1.0));
    EXPECT_DOUBLE_EQ(0.50000, scale(15.0, 20.0, 10.0, 1.0));
    EXPECT_DOUBLE_EQ(0.84375, scale(17.5, 20.0, 10.0, 1.0));
    EXPECT_DOUBLE_EQ(1.00000, scale(20.0, 20.0, 10.0, 1.0));
    EXPECT_DOUBLE_EQ(1.00000, scale(21.0, 20.0, 10.0, 1.0));

    EXPECT_DOUBLE_EQ(0.00000, scale( 9.0, 20.0, 10.0, 2.0));
    EXPECT_DOUBLE_EQ(0.00000, scale(10.0, 20.0, 10.0, 2.0));
    EXPECT_NEAR(     0.02441, scale(12.5, 20.0, 10.0, 2.0), 1e-5);
    EXPECT_DOUBLE_EQ(0.25000, scale(15.0, 20.0, 10.0, 2.0));
    EXPECT_NEAR(     0.71191, scale(17.5, 20.0, 10.0, 2.0), 1e-5);
    EXPECT_DOUBLE_EQ(1.00000, scale(20.0, 20.0, 10.0, 2.0));
    EXPECT_DOUBLE_EQ(1.00000, scale(21.0, 20.0, 10.0, 2.0));

    EXPECT_DOUBLE_EQ(1.00000, invScale( 9.0, 20.0, 10.0, 1.0));
    EXPECT_DOUBLE_EQ(1.00000, invScale(10.0, 20.0, 10.0, 1.0));
    EXPECT_DOUBLE_EQ(0.84375, invScale(12.5, 20.0, 10.0, 1.0));
    EXPECT_DOUBLE_EQ(0.50000, invScale(15.0, 20.0, 10.0, 1.0));
    EXPECT_DOUBLE_EQ(0.15625, invScale(17.5, 20.0, 10.0, 1.0));
    EXPECT_DOUBLE_EQ(0.00000, invScale(20.0, 20.0, 10.0, 1.0));
    EXPECT_DOUBLE_EQ(0.00000, invScale(21.0, 20.0, 10.0, 1.0));

    EXPECT_DOUBLE_EQ(1.00000, scaleInf( 9.0, 20.0, 10.0, 1.0));
    EXPECT_DOUBLE_EQ(1.00000, scaleInf(10.0, 20.0, 10.0, 1.0));
    EXPECT_NEAR(     1.18518, scaleInf(12.5, 20.0, 10.0, 1.0), 1e-5);
    EXPECT_DOUBLE_EQ(2.00000, scaleInf(15.0, 20.0, 10.0, 1.0));
    EXPECT_DOUBLE_EQ(6.40000, scaleInf(17.5, 20.0, 10.0, 1.0));
    EXPECT_EQ(INFINITY, scaleInf(20.0, 20.0, 10.0, 1.0));
    EXPECT_EQ(INFINITY, scaleInf(21.0, 20.0, 10.0, 1.0));

}


#pragma clang diagnostic pop