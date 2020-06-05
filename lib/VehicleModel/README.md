# Vehicle Model

A model to simulate a vehicle. 

## How to use

The vehicle model `VehicleModel` class contains algorithms to simulate vehicle dynamics. The following process describes how to use the model.

1. Before every simulation, perform a reset: `VehicleModel::reset()`.
2. Access the vehicle state `VehicleModel::getState()` and set the initial state (e.g. position, heading angle, velocity).
3. Run simulation steps:
    1. Set the input values by accessing the input fields `VehicleModel::getInput()`.
    2. Run `VehicleModel::step(double timeStepSize)`.
    3. Access the output (e.g. updated position, velocity, acceleration) `VehicleModel::getState()`
    
The inputs are designed to be linear (regarding drive force, brake torque and wheel angle) and can be set between -1 and 1. If you need a realistic steering, drive train or brake model, create a wrapper and implement it.

The model works best with a maximum time step size of 0.01 or less (100 Hz update rate). 10 Hz might lead to oscillations in combination with the controller.

## The pedal and steering controller

_TODO: move to src folder README_

To use the vehicle model by a driver model - or other intelligent driving functions -, you can use the controller class `PrimaryController` to generate the input based on a target value (e.g. desired acceleration, desired yaw rate, desired curvature, etc.). 

The controller can basically be used as the vehicle model (`reset()` and `step()`). However, the setup is slightly different:

Set the control variables by using `setVariables(double *value, double *target, double *output, double *offset = nullptr)`: 
    * `value`: A pointer to the actual control value (e.g. `&vehicle.getState()->a`);
    * `target`: The desired value for the control value (e.g. the desired acceleration of the driver model)
    * `output`: The output value to be controlled (e.g. `&vehicle.getState()->pedal`)
    * `offset`: An optional target output value, which is controlled by a P controller if set to a non infinite value directly.
    
Set the output value range `setRange(double lower, double upper, double maxChange)`. This function sets the lower and upper bounds for the output value and limits the absolute maximum change of the output value (derivative by time).

Set the control parameters `setParameters(double k_p, double k_i, double k_d, double o_p = 1.0)` for the PID controller. Optionally you can set the proportional parameter for the offset controller.

## Parameters

Usually for the yaw rate and for the acceleration (control values) you can try to start with 1.0, 0.0, 0.0 (parameters for P, I, D) and then change according to your needs.



 