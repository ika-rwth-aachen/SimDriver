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
// Created by Jens Klimke on 2020-06-05.
//


#ifndef SIM_DRIVER_CONTROLLER_H
#define SIM_DRIVER_CONTROLLER_H

#include "utils/Filter.h"


class _Controller {

protected:

    double *actualP = nullptr; //!< The compensatory proportional actual value
    double *targetP = nullptr; //!< The compensatory proportional target value
    double *actualI = nullptr; //!< The compensatory integral actual value
    double *targetI = nullptr; //!< The compensatory integral target value
    double *actualD = nullptr; //!< The compensatory derivative actual value
    double *targetD = nullptr; //!< The compensatory derivative target value

    double *preValue = nullptr; //!< The anticipatory pre-controlled value
    double *offset   = nullptr; //!< The offset value
    double *output   = nullptr; //!< The output value

    double k_P = 0.0; //!< Proportional parameter for compensation
    double k_I = 0.0; //!< Integral parameter for compensation
    double k_D = 0.0; //!< Derivative parameter for compensation
    double k_A = 1.0; //!< Proportional parameter for anticipation

    double range[2]  = {-1.0, 1.0}; //!< The valid range of the output value
    sim_driver::Filter _filter{}; //!< Filter container for the anticipatory control


public:


    /**
     * Perform a controller step with the given time step size
     * @param timeStepSize Time step size
     */
    bool step(double timeStepSize);

    /**
     * Sets the offset value
     * @param offset Offset value
     */
    void setOffset(double *offset);


    /**
     * Sets the anticipation controller variable
     * @param value Pre-control value
     * @param k Gain parameter
     * @param filterLength Anticipation filter length
     */
    void setAnticipation(double *value, double k, unsigned int filterLength = 1.0);


    /**
     * Sets the compensatory proportional controller variables
     * @param actual Actual value
     * @param target Target value
     * @param k Gain parameter
     */
    void setProportionalCompensation(double *actual, double *target, double k);


    /**
     * Sets the compensatory integral controller variables
     * @param actual Actual value
     * @param target Target value
     * @param k Gain parameter
     */
    void setIntegralCompensation(double *actual, double *target, double k);


    /**
     * Sets the compensatory derivative controller variables
     * @param actual Actual value
     * @param target Target value
     * @param k Gain parameter
     */
    void setDerivativeCompensation(double *actual, double *target, double k);


    /**
     * Sets the output variable and optionally the maximum and minimum value
     * @param output Output variable
     * @param max Maximum value
     * @param min Minimum value
     */
    void setOutput(double *output, double max = 1.0, double min = -1.0);



};


#endif //SIM_DRIVER_CONTROLLER_H
