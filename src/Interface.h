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
// 
// Created by Jens Klimke on 2020-04-07.
//


#ifndef AGENT_MODEL_INTERFACE_H
#define AGENT_MODEL_INTERFACE_H


namespace agent_model {

    static const unsigned int NOT = 32; //!< Maximum defined number of targets.
    static const unsigned int NOL = 32; //!< Maximum defined number of lanes.
    static const unsigned int NOS = 32; //!< Maximum defined number of signals.
    static const unsigned int NOH = 32; //!< Maximum defined number of horizon sample points.
    static const unsigned int NORP = 2; //!< Number of reference points per control path.
    static const unsigned int NOCP = 3; //!< Number of reference points per control path.
    static const unsigned int NOA = 32; //!< Number of auxiliary states.


    /*!< This enum describes a access state of an area. */
    enum Accessibility { ACC_NOT_SET, ACC_ACCESSIBLE, ACC_NOT_ALLOWED, ACC_NOT_ACCESSIBLE };

    /*!< This enum defines the driving direction of a lane. */
    enum DrivingDirection { DD_FORWARDS, DD_BACKWARDS, DD_BOTH, DD_NONE, DD_NOT_SET };

    /*!< This enum describes the type of a road signal. */
    enum SignalType { SIGNAL_NOT_SET, SIGNAL_STOP, SIGNAL_TLS, SIGNAL_SPEED_LIMIT, SIGNAL_YIELD, SIGNAL_PRIORITY };

    /*!< This enum describes the priority of a target. */
    enum TargetPriority { TARGET_ON_INTERSECTION, TARGET_ON_PRIORITY_LANE, TARGET_ON_GIVE_WAY_LANE, TARGET_PRIORITY_NOT_SET};

    /*!< This enum describes the maneuver performed by the host. */
    enum Maneuver { STRAIGHT, TURN_LEFT, TURN_RIGHT };



    /*!< A 2D position class. */
    struct Position {
        double x; //!< The x ordinate. (in *m*)
        double y; //!< The y ordinate. (in *m*)
        Position(): x(0.0), y(0.0) {}
        Position(double pX, double pY) : x(pX), y(pY) {}

    };

    /*!< A 2D position with motion and a influence factor. */
    struct DynamicPosition {
        double x; //!< The x ordinate. (in *m*)
        double y; //!< The y ordinate. (in *m*)
        double dx; //!< The derivative of the x component. (in *m/s*)
        double dy; //!< The derivative of the y component. (in *m/s*)
    };

    /*!< A point class, consisting of a distance and a value. */
    struct Point {
        double distance; //!< The distance at which the value applies. (in *m*)
        double time; //!< The relative time at which the value applies. (in *s*)
        double value; //!< The value of the point.
    };

    /*!< A dimensions class, saving width and length of an object. */
    struct Dimensions {
        double width; //!< The width of an object. (in *m*)
        double length; //!< The length of an object. (in *m*)
    };

    /*!< A class to save a vehicle state. */
    struct VehicleState {
        double v; //!< The velocity of the vehicle in x direction. (in *m/s*)
        double a; //!< The acceleration of the vehicle in x direction. (in *m/s^2*)
        double psi; //!< The yaw angle of the vehicle which is the angle between the vehicle x axis and heading of the current lane in mathematical positive direction. (in *rad*)
        double dPsi; //!< The time derivative of the yaw angle (yaw rate). (in *rad/s*)
        double s; //!< The distance, the vehicle travelled since the last reset. (in *m*)
        double d; //!< The lateral offset of the vehicle to the current reference line of the track (e.g. lane center). (in *m*)
        double pedal; //!< The actual pedal value [-1..1]. Negative values define a brake pedal
        double steering; //!< The actual steering value [-1..1]. Negative values define left turns
        Maneuver maneuver; //!< The general classification of the vehicle's path during the scenario
    };

    /*!< A class to store horizon points. */
    struct Horizon {
        double ds[NOH]; //!< Distance to the horizon point along s measured from the origin of the ego coordinate system. (in *m*)
        double x[NOH]; //!< x ordinate relative to ego unit (in *m*)
        double y[NOH]; //!< y ordinate relative to ego unit (in *m*)
        double psi[NOH]; //!< heading relative to vehicle x-axis (in *rad*)
        double kappa[NOH]; //!< curvature of the road (in *1/m*)
        double egoLaneWidth[NOH]; //!< Width of the ego lane (in *m*)
        double rightLaneWidth[NOH]; //!< Width of the right lane (in *m*)
        double leftLaneWidth[NOH]; //!< Width of the left lane (in *m*)
    };

    /*!< A class to store lane information. */
    struct Lane {
        int id; //!< Unique ID of the signal. The id is not just an identifier but also specifies the position of the lane relative to the ego lane in OpenDRIVE manner! e.g. -1 = the next lane to the left, 1 = the next lane to the right.
        double width; //!< Width of the lane (in *m*)
        double route; //!< Distance on the lane until the lane splits from the current route. (in *m*)
        double closed; //!< Distance on the lane until the lane is closed. (in *m*)
        DrivingDirection dir; //!< The driving direction of the lane related to the ego direction.
        Accessibility access; //!< The accessibility of the lane from the ego lane.
    };

    /*!< A class to store control path information */
    struct ControlPath {
        double offset; //!< The lateral offset from the reference line to be controlled to. (in *m*)
        double factor; //!< A factor to describe the influence of the point
        DynamicPosition refPoints[NORP]; //!< The reference points for the lateral control.
    };

    /*!< A class to store signal information. */
    struct Signal {
        unsigned int id; //!< Unique ID of the signal
        double ds; //!< Distance to the sign from the current position along the reference line. (in *m*)
        SignalType type; //!< Type of the signal.
        int value; //!< Value of the signal.
    };

    /*!< A class to store target information. */
    struct Target {
        unsigned int id; //!< Unique ID of the target. id=0 indicates that the target is not defined in the array position
        double ds; //!< Distance along s to the target center point from the ego driver position. (in *m*)
        Position xy; //!< Relative position of the target relative to the driver position and heading.
        double v; //!< Absolute velocity of the target. (in *m/s*)
        double a; //!< Absolute acceleration of the target. (in *m/s^2*)
        double d; //!< Lateral offset of the target in its corresponding lane. (in *m*)
        double psi; //!< Relative yaw angle of the target vehicle to the ego yaw angle. (in *rad*)
        int lane; //!< Lane ID of the actual lane of the target relative to driver's lane.
        Dimensions size; //!< Width and length of the target.
        double dsIntersection; //!< Distance along s to the intersection (if target is approaching an intersection)
        TargetPriority priority; //!< Priority of the target's lane. Used to determine right of way.
    };

    /*!< A class to store the internal state for the decision&#x2F;stopping component. */
    struct DecisionStopping {
        unsigned int id; //!< The ID of the stop.
        double position; //!< The absolute longitudinal position of the stop.
        double standingTime; //!< The time, the driver shall stand at the stop.
    };

    /*!< A class to store the internal state for the decisions components. */
    struct Decisions {
        int laneChange; //!< The decision to perform a lane change. The sign defines the direction. The value defines the number of lanes to be changed.
        Point lateral; //!< The decision to move to a defined lateral offset within a defined distance or time (mode=0: distance, mode=1: time).
        DecisionStopping stopping[NOS]; //!< The decision information for a stop.
    };

    /*!< A class to store the internal state for the conscious&#x2F;velocity component. */
    struct ConsciousVelocity {
        double local; //!< The local velocity. (in *m/s*)
        double prediction; //!< The prediction mean velocity. (in *m/s*)
    };

    /*!< A class to store the internal state for the conscious&#x2F;stop component. */
    struct ConsciousStop {
        double ds; //!< The actual distance to the stop point. (in *m*)
        double dsMax; //!< The reference distance at which the driver decides to stop (in *m*)
        bool standing; //!< A flag to define if the driver has stopped for the desired stop.
    };

    /*!< A class to store the internal state for the conscious&#x2F;follow component. */
    struct ConsciousFollow {
        double distance; //!< The distance to the target to be followed. (in *m*)
        double velocity; //!< The absolute velocity of the target to be followed. (in *m/s*)
        bool standing; //!< A flag to define whether the driver wants to keep the vehicle in standstill.
    };

    /*!< A class to store the internal state for the conscious&#x2F;lateral component. */
    struct ConsciousLateral {
        ControlPath paths[NOCP]; //!< An array of control paths
    };

    /*!< A class to store the internal state for the conscious components. */
    struct Conscious {
        ConsciousVelocity velocity; //!< A class to store the internal state for the conscious/velocity component.
        ConsciousStop stop; //!< A class to store the internal state for the conscious/stop component.
        ConsciousFollow follow; //!< A class to store the internal state for the conscious/follow component.
        ConsciousLateral lateral; //!< A class to store the internal state for the conscious/lateral component.
    };

    /*!< A class to store the internal state for the subconscious components. */
    struct Subconscious {
        double a; //!< Desired acceleration. (in *m/s^2*)
        double dPsi; //!< Desired yaw rate. (in *rad/s*)
        double kappa; //!< Desired curvature. (in *1/m*)
        double pedal; //!< Desired pedal value.
        double steering; //!< Desired steering angle.
    };

    /*!< A class to store all memory vehicle states. */
    struct MemoryVehicle {
        double s; //!< Absolute travelled distance since the last reset. (in *m*)
    };

    /*!< A class to store all memory lateral control states. */
    struct MemoryLateral {
        double time; //!< The time to reach the lateral offset.
        double startTime; //!< The start time of the lateral motion.
        double distance; //!< The distance to reach the lateral offset.
        double startDistance; //!< The start distance of the lateral motion.
        double offset; //!< The offset to be reached.
    };

    /*!< A class to store all memory lane change states. */
    struct MemoryLaneChange {
        int switchLane; //!< The lane to be switched to.
        int decision; //!< The decisions to which lane the driver wants to change to. 
        double startTime; //!< The start time of the lane change. (in *s*)
    };

    /*!< A class to store the parameters for velocity components. */
    struct ParameterVelocityControl {
        double thwMax; //!< The maximum time headway the driver starts to react (in *s*)
        double delta; //!< The power for the local speed reaction (see delta in IDM: https://en.wikipedia.org/wiki/Intelligent_driver_model)
        double deltaPred; //!< The power for the predictive speed reaction
        double a; //!< The maximum acceleration (in *m/s^2*)
        double b; //!< The maximum deceleration (in *m/s^2*)
        double vScale; //!< A scale factor to scale up or down the speed limit
        double ayMax; //!< Maximum lateral acceleration (in *m/s^2*)
        double vComfort; //!< Maximum personal comfortable velocity (in *m/s*)
    };

    /*!< A class to store the parameters for follow components. */
    struct ParameterFollowing {
        double timeHeadway; //!< The time headway the driver tries to reach during following (in *s*)
        double dsStopped; //!< The distance to the controlled target when stopped
        double thwMax; //!< The time headway the driver shall earliest react to follow (in *s*)
    };

    /*!< A class to store the parameters of the ego vehicle. */
    struct ParameterVehicle {
        Dimensions size; //!< Size of the ego vehicle
        Position pos; //!< The driver's position referenced to the center of the vehicle box defined by *size*
    };

    /*!< A class to store the parameters of the steering components. */
    struct ParameterSteering {
        double thw[NORP]; //!< The time headway of the reference points
        double dsMin[NORP]; //!< The minimim distance of the reference points
        double P[NORP]; //!< The P parameter of the controller
        double D[NORP]; //!< The D parameter of the controller
    };

    /*!< A class to store the parameters of the stop components. */
    struct ParameterStopping {
        double dsGap; //!< The gap between vehicle front and stop sign during a stop.
        double TMax; //!< Maximum time headway to react for stopping
        double dsMax; //!< Maximum distance to react for stopping
        double T; //!< A time headway to parameterize the dynamics of the approaching
        double tSign; //!< The time the driver stop at a stop sign
        double vStopped; //!< The velocity at which the driver expects the vehicle to have stopped.
        double pedalDuringStanding; //!< The pedal value, the driver controls during standing.
    };

    /*!< A class to store the parameters of the lane change components. */
    struct ParameterLaneChange {
        double bSafe; //!< A safe deceleration
        double aThreshold; //!< Acceleration threshold
        double politenessFactor; //!< Politeness factor
        double time; //!< Time to perform a lane change
    };

    /*!< A class to store the inputs. */
    struct Input {
        VehicleState vehicle; //!< The vehicle state.
        Horizon horizon; //!< The horizon.
        Signal signals[NOS]; //!< The signals.
        Lane lanes[NOL]; //!< The lanes.
        Target targets[NOT]; //!< The targets.
    };

    /*!< A class to store all internal states. */
    struct State {
        double simulationTime; //!< The actual simulation time
        Decisions decisions; //!< Decision states.
        Conscious conscious; //!< Conscious states.
        Subconscious subconscious; //!< Subconscious states.
        double aux[NOA]; //!< Auxiliary states.
    };

    /*!< A class to store all memory states. */
    struct Memory {
        MemoryVehicle vehicle; //!< The memory for vehicle states.
        double velocity; //!< The local maximum velocity.  (in *m/s*)
        MemoryLateral lateral; //!< The memory for lateral control components.
        MemoryLaneChange laneChange; //!< The memory for lane change components.
    };

    /*!< A class to store all parameters. */
    struct Parameters {
        ParameterVehicle vehicle; //!< Parameters for velocity components.
        ParameterLaneChange laneChange; //!< Parameters for lane change components.
        ParameterStopping stop; //!< Parameters for stop components.
        ParameterVelocityControl velocity; //!< Parameters for velocity components.
        ParameterFollowing follow; //!< Parameters for follow components.
        ParameterSteering steering; //!< Parameters for steering components.
    };


/**
 * @brief The agent model interface.
 * The class implements the data structure of the agent model, consisting of input, state, memory and parameters.
 */
class Interface {

public:

    typedef agent_model::Input Input;
    typedef agent_model::State State;
    typedef agent_model::Memory Memory;
    typedef agent_model::Parameters Parameters;


protected:

    Input _input{}; //!< The input of the agent model.
    State _state{}; //!< The state of the agent model.
    Memory _memory{}; //!< The memory of the agent model.
    Parameters _param{}; //!< The parameters of the agent model.

public:

    /** Default constructor */
    Interface() = default;

    /** Default destructor */
    virtual ~Interface() = default;


    /**
    * Returns the pointer for the _input structure of the model
    * @return The _input point
    */
    Input *getInput() {
        return &_input;
    }

    /**
    * Returns the const pointer for the _input structure of the model
    * @return The const _input point
    */
    const Input *getInput() const {
        return &_input;
    }


    /**
    * Returns the pointer for the _state structure of the model
    * @return The _state point
    */
    State *getState() {
        return &_state;
    }

    /**
    * Returns the const pointer for the _state structure of the model
    * @return The const _state point
    */
    const State *getState() const {
        return &_state;
    }


    /**
    * Returns the pointer for the _memory structure of the model
    * @return The _memory point
    */
    Memory *getMemory() {
        return &_memory;
    }

    /**
    * Returns the const pointer for the _memory structure of the model
    * @return The const _memory point
    */
    const Memory *getMemory() const {
        return &_memory;
    }


    /**
    * Returns the pointer for the _param structure of the model
    * @return The _param point
    */
    Parameters *getParameters() {
        return &_param;
    }

    /**
    * Returns the const pointer for the _param structure of the model
    * @return The const _param point
    */
    const Parameters *getParameters() const {
        return &_param;
    }



};

} // namespace

#endif // AGENT_MODEL_INTERFACE_H
