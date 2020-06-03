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
// Credits (models):
// [1] Treiber, Martin; Hennecke, Ansgar; Helbing, Dirk (2000), "Congested traffic states in empirical observations
//      and microscopic simulations", Physical Review E, 62 (2): 1805–1824, arXiv:cond-mat/0002177,
//      Bibcode:2000PhRvE..62.1805T, doi:10.1103/PhysRevE.62.1805
// [2] Salvucci, D. D., & Gray, R. (2004). A Two-Point Visual Control Model of Steering. Perception, 33(10), 1233–1248.
//      https://doi.org/10.1068/p5343
// [3] Kesting, Arne & Treiber, Martin & Helbing, Dirk. (2007), "General Lane-Changing Model MOBIL for Car-Following
//      Models", Transportation Research Record. 1999. 86-94. 10.3141/1999-10.
//      https://mtreiber.de/publications/MOBIL_TRB.pdf
//
//
// model_collection.h

#ifndef AGENT_MODEL_COLLECTION_H
#define AGENT_MODEL_COLLECTION_H

#include <limits>
#include <cmath>

namespace agent_model {

    /**
     * The reaction based on the current speed and the desired speed. The model is defined in
     * the free part of the IDM model. [1]
     *
     * Conditions:
     * 1. The actual velocity v must not be negative. A negative velocity leads to an exception (invalid argument).
     * 2. A negative target velocity vTarget leads to a result of 2.
     * 3. If the actual velocity v is much larger than target velocity (v >= 2 * vTarget), the result is 2.
     * 4. If the target velocity vTarget is infinity the result is 0.
     *
     *
     * @param v       Current velocity (in *m/s*)
     * @param vTarget The target (desired) velocity (in *m/s*)
     * @param delta   The parameter \delta (in -)
     * @return Return the cruise scale-down factor
     */
    double IDMSpeedReaction(double v, double vTarget, double delta);


    /**
     * Calculates the reaction on the current speed and the desired speed with respect to the oncoming and local
     * situation. The local situation is described by the parameter v0, which is the desired reference speed in case
     * of uninfluenced driving (the desired speed). The oncoming situation is described by
     *
     * @param v         The actual velocity
     * @param vTarget   The local target velocity
     * @param delta     The delta parameter (@see IDMSpeedReaction)
     * @param dsStep    The distance to the velocity step
     * @param vStep     The reference velocity at the velocity step
     * @param TMax      The maximum prediction time headway
     * @param deltaP    The intensity parameter for the prediction
     * @return Returns the reaction value
     */
    double speedReaction(double v, double vTarget, double delta, const double *vStep, const double *dsStep, double TMax,
                         double deltaP);


    /**
     * Calculates the reaction on vehicles during approaching and following. [1]
     *
     * @param ds    The actual net distance between vehicle and target (in *m*)
     * @param vPre  Velocity of the target vehicle (NOT the relative velocity, in *m/s*)
     * @param v     Velocity of the ego vehicle (in *m/s*)
     * @param T     Desired time headway (in *s*)
     * @param TMax  Maximum relevant time headway (in *s*)
     * @param s0    Desired distance when stopping (in *m*)
     * @return The resultant acceleration and the scale down factor for cruising
     */
    double IDMFollowReaction(double ds, double vPre, double v, double T, double s0, double a, double b);


    /**
     * Calculates the yaw rate dependent on the given reference point. [2]
     *
     * @param x The distance in the vehicle x direction
     * @param y The distance in the vehicle y direction
     * @param dx The actual velocity of the vehicle
     * @param dy The lateral velocity (derivative of y)
     * @param P The control parameter for the reference angle
     * @param D The control parameter for the reference angle derivative
     * @param theta The reference angle (will be set by function, for debugging)
     * @param dTheta The reference angle derivative (will be set by function, for debugging)
     * @return The resultant yaw rate
     */
    double SalvucciAndGray(double x, double y, double dx, double dy, double P, double D, double &theta, double &dTheta);


    /**
     * Calculates the acceleration by the Intelligent Driver Model (original). [1]
     *
     * @param v   Velocity of the agent [m/s]
     * @param v0  Reference velocity (the desired speed without any preceiding road participant) [m/s]
     * @param ds  Distance to the preceiding road participant [m]
     * @param dv  Relative velocity to the preceiding road participant (dv = vego - v_rp) [m/s]
     * @param T   Parameter time headway [s]
     * @param s0  Parameter stop distance [m]
     * @param ac  Reference acceleration (ac >= 0) [m/s^2]
     * @param bc  Reference deceleration (bc >= 0) [m/s^2]
     * @return
     */
    double IDMOriginal(double v, double v0, double ds, double dv, double T, double s0, double ac, double bc);


    /**
     * Calculates the safety criterion (safety factor > 0) and the incentive criterion (incentive factor > 0)
     * according to the MOBIL model [3]
     *
     * @param safety    The safety factor
     * @param incentive The incentive factor
     * @param v         Ego velocity (@see IDMOriginal)
     * @param v0        Desired ego velocity (@see IDMOriginal)
     * @param T         Time headway parameter (@see IDMOriginal)
     * @param s0        Stop distance parameter (@see IDMOriginal)
     * @param ac        Acceleration (@see IDMOriginal)
     * @param bc        Deceleration (@see IDMOriginal)
     * @param ds0f      Distance to the front vehicle on the original lane
     * @param v0f       Velocity to the front vehicle on the original lane
     * @param ds1f      Distance to the front vehicle on the target lane
     * @param v1f       Velocity to the front vehicle on the target lane
     * @param ds0b      Distance to the back vehicle on the original lane
     * @param v0b       Velocity to the back vehicle on the original lane
     * @param ds1b      Distance to the back vehicle on the target lane
     * @param v1b       Velocity to the back vehicle on the target lane
     * @param bSafe     Safe deceleration (bSafe > 0, e.g. 0.5) [m/s^2]
     * @param aThr      Threshold for accepted acceleration (aThr > 0, e.g. 0.5) [m/s^2]
     * @param p         Politeness factor, allowing to vary the motivation for lane-changing from purely egoistic to more cooperative driving behavior. (p > 0, e.g. 0.8) [-]
     */
    void
    MOBILOriginal(double &safety, double &incentive, double v, double v0, double T, double s0, double ac, double bc,
                  double ds0f, double v0f, double ds1f, double v1f, double ds0b, double v0b, double ds1b, double v1b,
                  double bSafe, double aThr, double p);


    /**
     * @brief Helper function: interpolation.
     *
     * Interpolates the value xx on the data x,y.
     *
     * @param xx The interpolation point
     * @param x Sample x point
     * @param y Sample y values
     * @param n Number of sample points given
     * @param extrapMode 0 = -inf/inf is returned, 1 = is extrapolating, other = returns the first/last value
     * @return Returns the interpolated value.
     */
    double interpolate(double xx, const double *x, const double *y, unsigned int n, int extrapMode = 1);


    /**
     * Calculates the polynomial y = 3 * x^2 - 2 * x^3 between x=[0..1]. The curve's derivations are equal to zero
     * at x=0 and x=1, while y=0 at x=0 and y=1 at x=1. x out of bounds are set to 0 and 1 respectively.
     * @param x Input value
     * @return Result
     */
    double scale(double x);


    /**
     * Calculates the linear scale factor between xMin and xMax. x = xMax leads to 1, while x = xMin leads to zero.
     * Values out of the bounds are 1 and 0, respectively.
     * @param x Input value
     * @param xMax Maximum value
     * @param xMin Minimum value
     * @return Result
     */
    double linScale(double x, double xMax, double xMin);


    /**
     * Calculates the scale factor between xMin and xMax. x = xMax leads to 1, while x = xMin leads to zero.
     * The derivations are zero at the bounds. Values out of the bounds are 1 and 0, respectively.
     * @param x Input value
     * @param xMax Maximum value
     * @param xMin Minimum value
     * @param delta Potential factor to push the curve towards the min or max value
     * @return Result
     */
    double scale(double x, double xMax, double xMin, double delta = 1.0);


    /**
     * Calculates the scale factor between xMin and xMax. x = xMax leads to zero, while x = xMin leads to 1.
     * The derivations are zero at the bounds. Values out of the bounds are 1 and 0, respectively.
     * @param x Input value
     * @param xMax Maximum value
     * @param xMin Minimum value
     * @param delta Potential factor to push the curve towards the min or max value
     * @return Result
     */
    double invScale(double x, double xMax, double xMin, double delta = 1.0);


    /**
     * Scales the input from inf (x = xMax) to 1 (x = xMin). The derivative is zero at x = xMin
     * @param x Input value. Values out of the bounds are inf and 1, respectively.
     * @param xMax Maximum value
     * @param xMin Minimum value
     * @param delta Potential factor to push the curve towards the min or max value
     * @return Result
     */
    double scaleInf(double x, double xMax, double xMin, double delta = 1.0);


}

#endif //AGENT_MODEL_COLLECTION_H
