// Copyright (c) 2020 Jens Klimke <jens.klimke@rwth-aachen.de>. All rights reserved.
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
// 
// Created by Jens Klimke on 2020-04-04.
//


#ifndef VEHICLE_MODEL_INTERFACE_H
#define VEHICLE_MODEL_INTERFACE_H



/**
 * @brief A vehicle model interface.
 */
class VehicleModel {

public:

    /** A class to store a x and y component. */
    struct Vector2 {
        double x = 0.0; // The x element (in *m*)
        double y = 0.0; // The y element (in *m*)
    };

    /** A class to store the input. */
    struct Input {
        double steer = 0.0; // The steering input [-1..1] (in *-*)
        double pedal = 0.0; // The pedal input [-1..1] (in *-*)
        double slope = 0.0; // The slope agle of the road (in *rad*)
    };

    /** A class to store the states. */
    struct State {
        Vector2 position = {0.0, 0.0}; // The actual position (in *m*)
        double ds = 0.0; // The distance travelled in the current time step (in *m*)
        double s = 0.0; // The distance travelled since the last reset (in *m*)
        double v = 0.0; // The actual velocity (in *m/s*)
        double a = 0.0; // The actual acceleration (in *m/s^2*)
        double psi = 0.0; // The actual yaw angle (in *rad*)
        double dPsi = 0.0; // The actual yaw rate (in *rad/s*)
        double delta = 0.0; // The actual wheel steer angle (in *rad*)
        double kappa = 0.0; // The actual curvature (in *1/m*)
        double ay = 0.0; // The actual lateral acceleration (in *m/s^2*)
        double force = 0.0; // The actual body force (in *N*)
    };

    /** A class to store the parameters. */
    struct Parameters {
        double steerTransmission = 0.5; // The steering transmission (input.steer -> state.delta) (in *-*)
        double wheelBase = 3.0; // The wheel base (in *m*)
        double cwA = 0.6; // The cw parameter multiplied with the front area (in *m^2*)
        double mass = 1.5e3; // The mass (in *kg*)
        double powerMax = 1.0e5; // Maximum drive power (in *W*)
        double forceMax = 1.5e4; // Maximum drive force (in *N*)
        double idle = 0.1; // The ratio of maximum power at no pedal (in *-*)
        double rollCoefficient[3] = {4.0 * 9.91e-3, 4.0 * 1.95e-5, 4.0 * 1.76e-9}; // The roll coefficients (polynomial parameters of v^0, v^1, v^2) (in *Unit*)
        Vector2 size = {5.0, 2.2}; // The size of the vehicle
        Vector2 driverPosition = {0.5, 0.5}; // The position of the driver related to the center
    };



protected:

    Input input{}; // The state of the model.
    State state{}; // The state of the model.
    Parameters parameters{}; // The parameters of the model.


public:

    /** Default constructor */
    VehicleModel() = default;


    /** Default destructor */
    virtual ~VehicleModel() = default;


    /** This method resets the vehicle memory */
    void reset();


    /** performs a simulation step
     * @param timeStepSize The simulation time step size
     */
    bool step(double timeStepSize);


    /**
    * Returns the pointer for the input structure of the model
    * @return The Input pointer
    */
    Input *getInput();


    /**
    * Returns the const pointer for the input structure of the model
    * @return The const Input point
    */
    const Input *getInput() const;


    /**
    * Returns the pointer for the state structure of the model
    * @return The State pointer
    */
    State *getState();


    /**
    * Returns the const pointer for the state structure of the model
    * @return The const State point
    */
    const State *getState() const;


    /**
    * Returns the pointer for the parameters structure of the model
    * @return The Parameters pointer
    */
    Parameters *getParameters();


    /**
    * Returns the const pointer for the parameters structure of the model
    * @return The const Parameters point
    */
    const Parameters *getParameters() const;


};

#endif // VEHICLE_MODEL_INTERFACE_H
