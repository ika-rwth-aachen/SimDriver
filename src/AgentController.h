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


#ifndef SIMDRIVER_AGENTCONTROLLER_H
#define SIMDRIVER_AGENTCONTROLLER_H


class AgentController {

protected:


protected:

    double *_value  = nullptr; //!< the actual value
    double *_target = nullptr; //!< the target value
    double *_offset = nullptr; //!< an offset to be controlled directly
    double *_y      = nullptr; //!< the output value

    double in = 0.0; //!< the integral error
    double u  = 0.0; //!< the error

    double k_P = 0.0; //!< proportional parameter for compensation
    double k_I = 0.0; //!< integral parameter for compensation
    double k_D = 0.0; //!< derivative parameter for compensation
    double o_P = 1.0; //!< proportional parameter for compensation

    double _range[2]  = {-1.0, 1.0}; //!< the valid range of the output value
    double _maxChange = 1.0;         //!< the maximum change of the control value (derivative of output value)

    bool _reset = false;  //!< reset flag


public:


    /**
     * Reset the controller states (except the output value)
     */
    void reset();


    /**
     * Perform a controller step with the given time step size
     * @param timeStepSize Time step size
     */
    bool step(double timeStepSize);


    /**
     * Set the controller variables
     * @param value The actual value
     * @param target The desired value
     * @param output The output value
     * @param offset An offset added to the output
     */
    void setVariables(double *value, double *target, double *output, double *offset = nullptr);


    /**
     * Sets the parameters
     * @param k_p Proportional parameter
     * @param k_i Integral parameter
     * @param k_d Derivative parameter
     * @param o_p Offset controller proportional parameter
     */
    void setParameters(double k_p, double k_i, double k_d, double o_p = 1.0);


    /**
     * Sets the range of the output value
     * @param lower Lower limit
     * @param upper Upper limit
     * @param macChange The maximum change of the output value
     */
    void setRange(double lower, double upper, double maxChange);



};


#endif //SIMDRIVER_AGENTCONTROLLER_H
