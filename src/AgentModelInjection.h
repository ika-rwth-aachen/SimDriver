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


#ifndef AGENT_MODEL_REGISTRATION_H
#define AGENT_MODEL_REGISTRATION_H

#include <injection/Injection.h>
#include "Interface.h"

namespace agent_model {

    struct __Position : public Injection<Position> {
        using Injection<Position>::operator =;
        Injection<double> x;
        Injection<double> y;
    };

    struct __DynamicPosition : public Injection<DynamicPosition> {
        using Injection<DynamicPosition>::operator =;
        Injection<double> x;
        Injection<double> y;
        Injection<double> dx;
        Injection<double> dy;
    };

    struct __Point : public Injection<Point> {
        using Injection<Point>::operator =;
        Injection<double> distance;
        Injection<double> time;
        Injection<double> value;
    };

    struct __Dimensions : public Injection<Dimensions> {
        using Injection<Dimensions>::operator =;
        Injection<double> width;
        Injection<double> length;
    };

    struct __VehicleState : public Injection<VehicleState> {
        using Injection<VehicleState>::operator =;
        Injection<double> v;
        Injection<double> a;
        Injection<double> psi;
        Injection<double> dPsi;
        Injection<double> s;
        Injection<double> d;
        Injection<double> pedal;
        Injection<double> steering;
    };

    struct __Horizon : public Injection<Horizon> {
        using Injection<Horizon>::operator =;
        Injection<double> ds[NOH];
        Injection<double> x[NOH];
        Injection<double> y[NOH];
        Injection<double> psi[NOH];
        Injection<double> kappa[NOH];
        Injection<double> egoLaneWidth[NOH];
        Injection<double> rightLaneWidth[NOH];
        Injection<double> leftLaneWidth[NOH];
    };

    struct __Lane : public Injection<Lane> {
        using Injection<Lane>::operator =;
        Injection<int> id;
        Injection<double> width;
        Injection<double> route;
        Injection<double> closed;
        Injection<DrivingDirection> dir;
        Injection<Accessibility> access;
    };

    struct __ControlPath : public Injection<ControlPath> {
        using Injection<ControlPath>::operator =;
        Injection<double> offset;
        Injection<double> factor;
        __DynamicPosition refPoints[NORP];
    };

    struct __Signal : public Injection<Signal> {
        using Injection<Signal>::operator =;
        Injection<unsigned int> id;
        Injection<double> ds;
        Injection<SignalType> type;
        Injection<int> value;
    };

    struct __Target : public Injection<Target> {
        using Injection<Target>::operator =;
        Injection<unsigned int> id;
        Injection<double> ds;
        Injection<double> v;
        Injection<double> a;
        Injection<double> d;
        Injection<double> psi;
        Injection<int> lane;
        __Position xy;
        __Dimensions size;
    };

    struct __DecisionStopping : public Injection<DecisionStopping> {
        using Injection<DecisionStopping>::operator =;
        Injection<unsigned int> id;
        Injection<double> position;
        Injection<double> standingTime;
    };

    struct __Decisions : public Injection<Decisions> {
        using Injection<Decisions>::operator =;
        Injection<int> laneChange;
        __Point lateral;
        __DecisionStopping stopping[NOS];
    };

    struct __ConsciousVelocity : public Injection<ConsciousVelocity> {
        using Injection<ConsciousVelocity>::operator =;
        Injection<double> local;
        Injection<double> prediction;
    };

    struct __ConsciousStop : public Injection<ConsciousStop> {
        using Injection<ConsciousStop>::operator =;
        Injection<double> ds;
        Injection<double> dsMax;
        Injection<bool> standing;
    };

    struct __ConsciousFollow : public Injection<ConsciousFollow> {
        using Injection<ConsciousFollow>::operator =;
        Injection<double> distance;
        Injection<double> velocity;
        Injection<bool> standing;
    };

    struct __ConsciousLateral : public Injection<ConsciousLateral> {
        using Injection<ConsciousLateral>::operator =;
        __ControlPath paths[NOCP];
    };

    struct __Conscious : public Injection<Conscious> {
        using Injection<Conscious>::operator =;
        __ConsciousVelocity velocity;
        __ConsciousStop stop;
        __ConsciousFollow follow;
        __ConsciousLateral lateral;
    };

    struct __Subconscious : public Injection<Subconscious> {
        using Injection<Subconscious>::operator =;
        Injection<double> a;
        Injection<double> dPsi;
        Injection<double> kappa;
        Injection<double> pedal;
        Injection<double> steering;
    };

    struct __MemoryVehicle : public Injection<MemoryVehicle> {
        using Injection<MemoryVehicle>::operator =;
        Injection<double> s;
    };

    struct __MemoryLateral : public Injection<MemoryLateral> {
        using Injection<MemoryLateral>::operator =;
        Injection<double> time;
        Injection<double> startTime;
        Injection<double> distance;
        Injection<double> startDistance;
        Injection<double> offset;
    };

    struct __MemoryLaneChange : public Injection<MemoryLaneChange> {
        using Injection<MemoryLaneChange>::operator =;
        Injection<int> switchLane;
        Injection<int> decision;
        Injection<double> startTime;
    };

    struct __ParameterVelocityControl : public Injection<ParameterVelocityControl> {
        using Injection<ParameterVelocityControl>::operator =;
        Injection<double> thwMax;
        Injection<double> delta;
        Injection<double> deltaPred;
        Injection<double> a;
        Injection<double> b;
        Injection<double> vScale;
        Injection<double> ayMax;
        Injection<double> vComfort;
    };

    struct __ParameterFollowing : public Injection<ParameterFollowing> {
        using Injection<ParameterFollowing>::operator =;
        Injection<double> timeHeadway;
        Injection<double> dsStopped;
        Injection<double> thwMax;
    };

    struct __ParameterVehicle : public Injection<ParameterVehicle> {
        using Injection<ParameterVehicle>::operator =;
        __Dimensions size;
        __Position pos;
    };

    struct __ParameterSteering : public Injection<ParameterSteering> {
        using Injection<ParameterSteering>::operator =;
        Injection<double> thw[NORP];
        Injection<double> dsMin[NORP];
        Injection<double> P[NORP];
        Injection<double> D[NORP];
    };

    struct __ParameterStopping : public Injection<ParameterStopping> {
        using Injection<ParameterStopping>::operator =;
        Injection<double> dsGap;
        Injection<double> TMax;
        Injection<double> dsMax;
        Injection<double> T;
        Injection<double> tSign;
        Injection<double> vStopped;
        Injection<double> pedalDuringStanding;
    };

    struct __ParameterLaneChange : public Injection<ParameterLaneChange> {
        using Injection<ParameterLaneChange>::operator =;
        Injection<double> bSafe;
        Injection<double> aThreshold;
        Injection<double> politenessFactor;
        Injection<double> time;
    };

    struct __Input : public Injection<Input> {
        using Injection<Input>::operator =;
        __VehicleState vehicle;
        __Horizon horizon;
        __Signal signals[NOS];
        __Lane lanes[NOL];
        __Target targets[NOT];
    };

    struct __State : public Injection<State> {
        using Injection<State>::operator =;
        Injection<double> simulationTime;
        Injection<double> aux[NOA];
        __Decisions decisions;
        __Conscious conscious;
        __Subconscious subconscious;
    };

    struct __Memory : public Injection<Memory> {
        using Injection<Memory>::operator =;
        Injection<double> velocity;
        __MemoryVehicle vehicle;
        __MemoryLateral lateral;
        __MemoryLaneChange laneChange;
    };

    struct __Parameters : public Injection<Parameters> {
        using Injection<Parameters>::operator =;
        __ParameterVehicle vehicle;
        __ParameterLaneChange laneChange;
        __ParameterStopping stop;
        __ParameterVelocityControl velocity;
        __ParameterFollowing follow;
        __ParameterSteering steering;
    };


    void registerTree(__Position *tree, Position *data, const void *owner);
    void registerTree(__DynamicPosition *tree, DynamicPosition *data, const void *owner);
    void registerTree(__Point *tree, Point *data, const void *owner);
    void registerTree(__Dimensions *tree, Dimensions *data, const void *owner);
    void registerTree(__VehicleState *tree, VehicleState *data, const void *owner);
    void registerTree(__Horizon *tree, Horizon *data, const void *owner);
    void registerTree(__Lane *tree, Lane *data, const void *owner);
    void registerTree(__ControlPath *tree, ControlPath *data, const void *owner);
    void registerTree(__Signal *tree, Signal *data, const void *owner);
    void registerTree(__Target *tree, Target *data, const void *owner);
    void registerTree(__DecisionStopping *tree, DecisionStopping *data, const void *owner);
    void registerTree(__Decisions *tree, Decisions *data, const void *owner);
    void registerTree(__ConsciousVelocity *tree, ConsciousVelocity *data, const void *owner);
    void registerTree(__ConsciousStop *tree, ConsciousStop *data, const void *owner);
    void registerTree(__ConsciousFollow *tree, ConsciousFollow *data, const void *owner);
    void registerTree(__ConsciousLateral *tree, ConsciousLateral *data, const void *owner);
    void registerTree(__Conscious *tree, Conscious *data, const void *owner);
    void registerTree(__Subconscious *tree, Subconscious *data, const void *owner);
    void registerTree(__MemoryVehicle *tree, MemoryVehicle *data, const void *owner);
    void registerTree(__MemoryLateral *tree, MemoryLateral *data, const void *owner);
    void registerTree(__MemoryLaneChange *tree, MemoryLaneChange *data, const void *owner);
    void registerTree(__ParameterVelocityControl *tree, ParameterVelocityControl *data, const void *owner);
    void registerTree(__ParameterFollowing *tree, ParameterFollowing *data, const void *owner);
    void registerTree(__ParameterVehicle *tree, ParameterVehicle *data, const void *owner);
    void registerTree(__ParameterSteering *tree, ParameterSteering *data, const void *owner);
    void registerTree(__ParameterStopping *tree, ParameterStopping *data, const void *owner);
    void registerTree(__ParameterLaneChange *tree, ParameterLaneChange *data, const void *owner);
    void registerTree(__Input *tree, Input *data, const void *owner);
    void registerTree(__State *tree, State *data, const void *owner);
    void registerTree(__Memory *tree, Memory *data, const void *owner);
    void registerTree(__Parameters *tree, Parameters *data, const void *owner);


    template<typename T, typename D>
    void registerStructArray(T *pointer, D *data, const void *owner, std::vector<unsigned long> sizes, unsigned long j = 0) {

        // last iteration
        if(j == sizes.size() - 1) {

            // iterate over j-th size
            for(unsigned long i = 0; i < sizes[j]; ++i) {
                registerTree(pointer, data, owner);
                pointer++; data++;
            }

            return;

        }

        // calculate step size
        unsigned long steps = 1;
        for(unsigned long i = j + 1; i < sizes.size(); ++i)
            steps *= sizes[i];

        // iterate over j-th size
        for(unsigned long i = 0; i < sizes[j]; ++i) {
            registerStructArray(pointer, data, owner, sizes, j + 1);
            pointer += steps;
            data += steps;
        }

    }

    template<typename T, typename D>
    void registerArray(T *pointer, D *data, const void *owner, std::vector<unsigned long> sizes, unsigned long j = 0) {

        // last iteration
        if(j == sizes.size() - 1) {

            // iterate over j-th size
            for(unsigned long i = 0; i < sizes[j]; ++i) {
                pointer->registerValue(data, owner);
                pointer++; data++;
            }

            return;

        }

        // calculate step size
        unsigned long steps = 1;
        for(unsigned long i = j + 1; i < sizes.size(); ++i)
            steps *= sizes[i];

        // iterate over j-th size
        for(unsigned long i = 0; i < sizes[j]; ++i) {
            registerArray(pointer, data, owner, sizes, j + 1);
            pointer += steps;
            data += steps;
        }

    }


} // namespace

#endif // AGENT_MODEL_REGISTRATION_H
