// Copyright (c) 2020 Jens Klimke.
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
// Created by Jens Klimke on 2020-10-23.
// Contributors:
//

#ifndef SIM_DRIVER_MATH_H
#define SIM_DRIVER_MATH_H

#include <cmath>
#include <stdexcept>
#include <algorithm>

namespace sim_driver::math {

    /**
     * @brief Helper function: interpolation.
     *
     * Interpolates the value xx on the data x,y.
     *
     * @param xx The interpolation point
     * @param x Sample x point
     * @param y Sample y values
     * @param n Number of sample points given
     * @param extrapolationMode 0 = -inf/inf is returned, 1 = is extrapolating, other = returns the first/last value
     * @return Returns the interpolated value.
     */
    inline double interpolate(double xx, const double *x, const double *y, unsigned int n, int extrapolationMode = 1) {

        using namespace std;

        // instantiate
        unsigned long i1;
        unsigned long i0 = n; // last finite value

        for (i1 = 0; i1 < n; ++i1) {

            // ignore inf values
            if (isinf(x[i1]))
                continue;
            else
                i0 = i1;

            // point before current sample point
            if (x[i1] > xx)
                break;

        }

        // reset behind last valid value
        if (i0 != n && (i1 == n || isinf(x[i1])))
            i1 = i0 + 1;

        bool e = i1 == n || isinf(x[i1]);
        bool s = i1 == 0 || isinf(x[i1 - 1]);

        // can not find any solution
        if (e && std::abs(x[n - 1] - xx) < 1e-15) {

            return y[n - 1];

        } else if (s && std::abs(x[0] - xx) < 1e-15) {

            return y[0];

        } else if (s) {

            if (extrapolationMode == 0)
                return -1.0 * INFINITY;
            else if (extrapolationMode == 1 && i1 != n)
                i1++;
            else if (extrapolationMode == 2)
                return y[i1];

        } else if (e) {

            if (extrapolationMode == 0)
                return INFINITY;
            else if (extrapolationMode == 1)
                i1--;
            else if (extrapolationMode == 2)
                return y[i1 - 1];

        }

        // check validity
        if (i1 == 0 || i1 == n || x[i1 - 1] >= x[i1])
            throw std::invalid_argument("interpolation not possible.");


        // interpolate linearly
        i0 = i1 - 1;
        return y[i0] + (xx - x[i0]) * (y[i1] - y[i0]) / (x[i1] - x[i0]);

    }


    /**
     * Calculates the polynomial y = 3 * x^2 - 2 * x^3 between x=[0..1]. The curve's derivations are equal to zero
     * at x=0 and x=1, while y=0 at x=0 and y=1 at x=1. x out of bounds are set to 0 and 1 respectively.
     * @param x Input value
     * @return Result
     */
    inline double scale(double x) {

        using namespace std;

        x = max(0.0, min(1.0, x));
        return 3 * x * x - 2 * x * x * x;

    }


    /**
     * Calculates the linear scale factor between xMin and xMax. x = xMax leads to 1, while x = xMin leads to zero.
     * Values out of the bounds are 1 and 0, respectively.
     * @param x Input value
     * @param xMax Maximum value
     * @param xMin Minimum value
     * @return Result
     */
    inline double linScale(double x, double xMax, double xMin) {

        return std::max(0.0, std::min(1.0, (x - xMin) / (xMax - xMin)));

    }


    /**
     * Calculates the scale factor between xMin and xMax. x = xMax leads to 1, while x = xMin leads to zero.
     * The derivations are zero at the bounds. Values out of the bounds are 1 and 0, respectively.
     * @param x Input value
     * @param xMax Maximum value
     * @param xMin Minimum value
     * @param delta Potential factor to push the curve towards the min or max value
     * @return Result
     */
    inline double scale(double x, double xMax, double xMin, double delta = 1.0) {

        // limit delta
        delta = std::max(0.0, delta);

        // step at > 0.0
        if(delta == 0.0)
            return x <= xMin ? 0.0 : 1.0;

        // calculate scale
        auto s = scale(linScale(x, xMax, xMin));

        if(delta < 1.0)
            return 1.0 - pow(1.0 - s, 1.0 / delta); // inverted power
        else
            return pow(s, delta); // normal power

    }


    /**
     * Calculates the scale factor between xMin and xMax. x = xMax leads to zero, while x = xMin leads to 1.
     * The derivations are zero at the bounds. Values out of the bounds are 1 and 0, respectively.
     * @param x Input value
     * @param xMax Maximum value
     * @param xMin Minimum value
     * @param delta Potential factor to push the curve towards the min or max value
     * @return Result
     */
    inline double invScale(double x, double xMax, double xMin, double delta = 1.0) {

        return pow(scale((xMax - x) / (xMax - xMin)), delta);

    }


    /**
     * Scales the input from inf (x = xMax) to 1 (x = xMin). The derivative is zero at x = xMin
     * @param x Input value. Values out of the bounds are inf and 1, respectively.
     * @param xMax Maximum value
     * @param xMin Minimum value
     * @param delta Potential factor to push the curve towards the min or max value
     * @return Result
     */
    inline double scaleInf(double x, double xMax, double xMin, double delta = 1.0) {

        return 1.0 / invScale(x, xMax, xMin, delta);

    }

}

#endif // SIM_DRIVER_MATH_H
