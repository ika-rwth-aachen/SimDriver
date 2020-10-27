// Copyright (c) 2020 Institute for Automotive Engineering (ika), RWTH Aachen University. All rights reserved.
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
// model_collection.cpp

#include <stdexcept>
#include <cmath>
#include <SimDriver/model_collection.h>
#include <SimDriver/utils/Math.h>

namespace sim_driver::models {


    double IDMSpeedReaction(double v, double vTarget, double delta) {

        using namespace std;

        // v must not be negative or inf
        if (v < 0.0)
            throw invalid_argument("actual velocity must not be negative.");
        else if (isinf(v))
            throw invalid_argument("actual velocity must be finite.");

        // vTarget must not be negative
        if (vTarget < 0)
            throw invalid_argument("target velocity must not be negative.");

        // special cases
        if (vTarget <= 0.0 || v >= 2 * vTarget)
            return 2.0;
        else if (isinf(vTarget))
            return 0.0;

        // calculate result
        auto dv = vTarget - v;
        double r = pow(1.0 - std::abs(dv) / vTarget, delta);

        // switch for dv < 0
        return dv < 0.0 ? 2.0 - r : r;

    }


    double speedReaction(double v, double vTarget, double delta, const double *vStep, const double *dsStep, double TMax,
                         double deltaP) {

        using namespace std;
        using namespace sim_driver::math;
        using namespace sim_driver::models;

        // calculate local reaction
        auto local = sim_driver::models::IDMSpeedReaction(v, vTarget, delta);

        // max distance
        auto dsMax = v * TMax;

        // calculate factors
        double f0 = sim_driver::math::scale(dsStep[0], dsMax, 0.0, deltaP);
        double f1 = sim_driver::math::scale(dsStep[1], dsMax, 0.0, deltaP);

        // calculate reaction
        auto r0 = IDMSpeedReaction(v, vStep[0], delta);
        auto r1 = IDMSpeedReaction(v, vStep[1], delta);

        // calculate sum of reaction
        return f0 * f1 * local + (1.0 - f0) * r0 + (1.0 - f1) * r1;

    }


    double IDMFollowReaction(double ds, double vPre, double v, double T, double s0, double a, double b) {

        using namespace std;

        // v must not be negative or inf
        if (v < 0.0)
            throw invalid_argument("actual velocity must not be negative.");
        else if (isinf(v))
            throw invalid_argument("actual velocity must be finite.");

        // vTarget must not be negative
        if (vPre < 0)
            throw invalid_argument("target velocity must not be negative.");

        // avoid division by inf
        if (isinf(ds))
            return 0.0;

        // get rel. velocity and dsStar (IDM)
        auto dv = v - vPre;
        auto dsStar = s0 + v * T + 0.5 * dv * v / sqrt(a * -b);

        // avoid 0/0
        if (dsStar == 0.0 && ds == 0.0)
            return 1.0;

        // avoid negative distances
        if (ds <= 0.0)
            ds = 0.0;

        // return squared ratio
        return pow(dsStar / ds, 2.0);

    }


    double SalvucciAndGray(double x, double y, double dx, double dy, double P, double D,
            double &theta, double &dTheta) {

        using namespace std;

        // avoid problem with x=0, x=inf, ...
        if (isinf(x) || isinf(y) || y == 0) {
            theta = 0.0;
            dTheta = 0.0;
            return 0.0;
        }

        // calculate theta and dTheta
        theta = atan2(y, x);
        dTheta = (y * dx + x * dy) / (x * x + y * y);

        // calculate reaction
        return P * theta + D * dTheta;

    }


    double IDMOriginal(double v, double v0, double ds, double dv, double T, double s0, double ac, double bc) {

        using namespace std;

        // calculate acceleration
        auto s_star = s0 + v * T + (v * dv / (2.0 * sqrt(ac * bc)));
        auto acc = ac * (1.0 - pow(v / v0, 4) - pow(s_star / ds, 2));

        // check for nan or inf
        if (isnan(acc) || isinf(acc))
            acc = 0.0;

        return acc;

    }


    void MOBILOriginal(double &safety, double &incentive, double v, double v0, double T, double s0, double ac,
                       double bc, double ds0f, double v0f, double ds1f, double v1f, double ds0b, double v0b,
                       double ds1b, double v1b, double bSafe, double aThr, double p) {

        auto a00m = IDMOriginal(v, v0, ds0f, v - v0f, T, s0, ac, bc);          // acc(M)
        auto a11m = IDMOriginal(v, v0, ds1f, v - v1f, T, s0, ac, bc);          // acc'(M')
        auto a00b = IDMOriginal(v, v0, -ds0b, v0b - v, T, s0, ac, bc);         // acc(B)
        auto a01b = IDMOriginal(v, v0, ds1f - ds1b, v1b - v1f, T, s0, ac, bc); // acc(B')
        auto a10b = IDMOriginal(v, v0, ds0f - ds0b, v0b - v0f, T, s0, ac, bc); // acc'(B)
        auto a11b = IDMOriginal(v, v0, -ds1b, v1f - v, T, s0, ac, bc);         // acc'(B')

        /*
         * Original criteria:
         * a11b > -bSave,                                        // safety criterion
         * a11m - a00m > p * (a00b + a01b - a10b - a11b) + aThr  // incentive criterion
         */

        // save safety criterion
        safety = (a11b + bSafe) / bSafe;

        // return incentive criterion
        incentive = (a11m - a00m - p * (a00b + a01b - a10b - a11b) - aThr) / aThr;

    }

}