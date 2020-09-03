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
        double steer = 0.0;     // The steering input [-1..1] (in -)
        double pedal = 0.0;     // The pedal input [-1..1] (in -)
    };

    /** A class to store the states. */
    struct State {
        Vector2 position = {0.0, 0.0};  // The actual position (in *m*)
        double ds = 0.0;                // The distance travelled in the current time step (in *m*)
        double s = 0.0;                 // The distance travelled since the last reset (in *m*)
        double v = 0.0;                 // The actual velocity (in *m/s*)
        double a = 0.0;                 // The actual acceleration (in *m/s^2*)
        double psi = 0.0;               // The actual yaw angle (in *rad*)
        double dPsi = 0.0;              // The actual yaw rate (in *rad/s*)
        double delta = 0.0;             // The actual wheel steer angle (in *rad*)
        double kappa = 0.0;             // The actual curvature (in *1/m*)
    };

    /** A class to store the parameters. */
    struct Parameters {
        double maxWheelAngle{};         // Maximum steering angle (equal to virtual steering transmission, in *rad*)
        double wheelBase{};             // Wheel base (in *m*)
        double maxRelDrivePower{};      // Maximum relative drive power (in *W/kg* or *m^2/s^3*)
        double maxDriveAcc{};           // Maximum drive acceleration (in *m/s^2*)
        double maxBrakeAcc{};           // Maximum brake acceleration (in *m/s^2*)
        double idlePedal{};             // Pedal when no pedal input (in -)
        double longExternalAcc[3]{};    // Longitudinal long. acceleration (polynomial parameters of v^0, v^1, v^2) (in -)
        double sideDrift{};             // The side drift factor (ratio of lateral to longitudinal movement, in -)
    };



protected:

    Input input{};              // The state of the model.
    State state{};              // The state of the model.
    Parameters parameters{};    // The parameters of the model.


public:

    /**
     * Default constructor
     */
    VehicleModel() = default;


    /**
     * Default destructor
     */
    virtual ~VehicleModel() = default;


    /**
     * This method resets the vehicle memory
     */
    void reset();


    /**
     * Performs a simulation step
     * @param timeStepSize The simulation time step size
     */
    void step(double timeStepSize);


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
