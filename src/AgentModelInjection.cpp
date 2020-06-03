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


#include "AgentModelInjection.h"

namespace agent_model {


    void registerTree(__Position *tree, Position *data, const void *owner) {
        tree->registerValue(data, owner);
        tree->x.registerValue(&data->x, owner);
        tree->y.registerValue(&data->y, owner);
    }

    void registerTree(__DynamicPosition *tree, DynamicPosition *data, const void *owner) {
        tree->registerValue(data, owner);
        tree->x.registerValue(&data->x, owner);
        tree->y.registerValue(&data->y, owner);
        tree->dx.registerValue(&data->dx, owner);
        tree->dy.registerValue(&data->dy, owner);
    }

    void registerTree(__Point *tree, Point *data, const void *owner) {
        tree->registerValue(data, owner);
        tree->distance.registerValue(&data->distance, owner);
        tree->time.registerValue(&data->time, owner);
        tree->value.registerValue(&data->value, owner);
    }

    void registerTree(__Dimensions *tree, Dimensions *data, const void *owner) {
        tree->registerValue(data, owner);
        tree->width.registerValue(&data->width, owner);
        tree->length.registerValue(&data->length, owner);
    }

    void registerTree(__VehicleState *tree, VehicleState *data, const void *owner) {
        tree->registerValue(data, owner);
        tree->v.registerValue(&data->v, owner);
        tree->a.registerValue(&data->a, owner);
        tree->psi.registerValue(&data->psi, owner);
        tree->dPsi.registerValue(&data->dPsi, owner);
        tree->s.registerValue(&data->s, owner);
        tree->d.registerValue(&data->d, owner);
        tree->pedal.registerValue(&data->pedal, owner);
        tree->steering.registerValue(&data->steering, owner);
    }

    void registerTree(__Horizon *tree, Horizon *data, const void *owner) {
        tree->registerValue(data, owner);
        registerArray(&tree->ds[0], &data->ds[0], owner, {NOH});
        registerArray(&tree->x[0], &data->x[0], owner, {NOH});
        registerArray(&tree->y[0], &data->y[0], owner, {NOH});
        registerArray(&tree->psi[0], &data->psi[0], owner, {NOH});
        registerArray(&tree->kappa[0], &data->kappa[0], owner, {NOH});
        registerArray(&tree->egoLaneWidth[0], &data->egoLaneWidth[0], owner, {NOH});
        registerArray(&tree->rightLaneWidth[0], &data->rightLaneWidth[0], owner, {NOH});
        registerArray(&tree->leftLaneWidth[0], &data->leftLaneWidth[0], owner, {NOH});
    }

    void registerTree(__Lane *tree, Lane *data, const void *owner) {
        tree->registerValue(data, owner);
        tree->id.registerValue(&data->id, owner);
        tree->width.registerValue(&data->width, owner);
        tree->route.registerValue(&data->route, owner);
        tree->closed.registerValue(&data->closed, owner);
        tree->dir.registerValue(&data->dir, owner);
        tree->access.registerValue(&data->access, owner);
    }

    void registerTree(__ControlPath *tree, ControlPath *data, const void *owner) {
        tree->registerValue(data, owner);
        tree->offset.registerValue(&data->offset, owner);
        tree->factor.registerValue(&data->factor, owner);
        registerStructArray(&tree->refPoints[0], &data->refPoints[0], owner, {NORP});
    }

    void registerTree(__Signal *tree, Signal *data, const void *owner) {
        tree->registerValue(data, owner);
        tree->id.registerValue(&data->id, owner);
        tree->ds.registerValue(&data->ds, owner);
        tree->type.registerValue(&data->type, owner);
        tree->value.registerValue(&data->value, owner);
    }

    void registerTree(__Target *tree, Target *data, const void *owner) {
        tree->registerValue(data, owner);
        tree->id.registerValue(&data->id, owner);
        tree->ds.registerValue(&data->ds, owner);
        registerTree(&tree->xy, &data->xy, owner);
        tree->v.registerValue(&data->v, owner);
        tree->a.registerValue(&data->a, owner);
        tree->d.registerValue(&data->d, owner);
        tree->psi.registerValue(&data->psi, owner);
        tree->lane.registerValue(&data->lane, owner);
        registerTree(&tree->size, &data->size, owner);
    }

    void registerTree(__DecisionStopping *tree, DecisionStopping *data, const void *owner) {
        tree->registerValue(data, owner);
        tree->id.registerValue(&data->id, owner);
        tree->position.registerValue(&data->position, owner);
        tree->standingTime.registerValue(&data->standingTime, owner);
    }

    void registerTree(__Decisions *tree, Decisions *data, const void *owner) {
        tree->registerValue(data, owner);
        tree->laneChange.registerValue(&data->laneChange, owner);
        registerTree(&tree->lateral, &data->lateral, owner);
        registerStructArray(&tree->stopping[0], &data->stopping[0], owner, {NOS});
    }

    void registerTree(__ConsciousVelocity *tree, ConsciousVelocity *data, const void *owner) {
        tree->registerValue(data, owner);
        tree->local.registerValue(&data->local, owner);
        tree->prediction.registerValue(&data->prediction, owner);
    }

    void registerTree(__ConsciousStop *tree, ConsciousStop *data, const void *owner) {
        tree->registerValue(data, owner);
        tree->ds.registerValue(&data->ds, owner);
        tree->dsMax.registerValue(&data->dsMax, owner);
        tree->standing.registerValue(&data->standing, owner);
    }

    void registerTree(__ConsciousFollow *tree, ConsciousFollow *data, const void *owner) {
        tree->registerValue(data, owner);
        tree->distance.registerValue(&data->distance, owner);
        tree->velocity.registerValue(&data->velocity, owner);
        tree->standing.registerValue(&data->standing, owner);
    }

    void registerTree(__ConsciousLateral *tree, ConsciousLateral *data, const void *owner) {
        tree->registerValue(data, owner);
        registerStructArray(&tree->paths[0], &data->paths[0], owner, {NOCP});
    }

    void registerTree(__Conscious *tree, Conscious *data, const void *owner) {
        tree->registerValue(data, owner);
        registerTree(&tree->velocity, &data->velocity, owner);
        registerTree(&tree->stop, &data->stop, owner);
        registerTree(&tree->follow, &data->follow, owner);
        registerTree(&tree->lateral, &data->lateral, owner);
    }

    void registerTree(__Subconscious *tree, Subconscious *data, const void *owner) {
        tree->registerValue(data, owner);
        tree->a.registerValue(&data->a, owner);
        tree->dPsi.registerValue(&data->dPsi, owner);
        tree->kappa.registerValue(&data->kappa, owner);
        tree->pedal.registerValue(&data->pedal, owner);
        tree->steering.registerValue(&data->steering, owner);
    }

    void registerTree(__MemoryVehicle *tree, MemoryVehicle *data, const void *owner) {
        tree->registerValue(data, owner);
        tree->s.registerValue(&data->s, owner);
    }

    void registerTree(__MemoryLateral *tree, MemoryLateral *data, const void *owner) {
        tree->registerValue(data, owner);
        tree->time.registerValue(&data->time, owner);
        tree->startTime.registerValue(&data->startTime, owner);
        tree->distance.registerValue(&data->distance, owner);
        tree->startDistance.registerValue(&data->startDistance, owner);
        tree->offset.registerValue(&data->offset, owner);
    }

    void registerTree(__MemoryLaneChange *tree, MemoryLaneChange *data, const void *owner) {
        tree->registerValue(data, owner);
        tree->switchLane.registerValue(&data->switchLane, owner);
        tree->decision.registerValue(&data->decision, owner);
        tree->startTime.registerValue(&data->startTime, owner);
    }

    void registerTree(__ParameterVelocityControl *tree, ParameterVelocityControl *data, const void *owner) {
        tree->registerValue(data, owner);
        tree->thwMax.registerValue(&data->thwMax, owner);
        tree->delta.registerValue(&data->delta, owner);
        tree->deltaPred.registerValue(&data->deltaPred, owner);
        tree->a.registerValue(&data->a, owner);
        tree->b.registerValue(&data->b, owner);
        tree->vScale.registerValue(&data->vScale, owner);
        tree->ayMax.registerValue(&data->ayMax, owner);
        tree->vComfort.registerValue(&data->vComfort, owner);
    }

    void registerTree(__ParameterFollowing *tree, ParameterFollowing *data, const void *owner) {
        tree->registerValue(data, owner);
        tree->timeHeadway.registerValue(&data->timeHeadway, owner);
        tree->dsStopped.registerValue(&data->dsStopped, owner);
        tree->thwMax.registerValue(&data->thwMax, owner);
    }

    void registerTree(__ParameterVehicle *tree, ParameterVehicle *data, const void *owner) {
        tree->registerValue(data, owner);
        registerTree(&tree->size, &data->size, owner);
        registerTree(&tree->pos, &data->pos, owner);
    }

    void registerTree(__ParameterSteering *tree, ParameterSteering *data, const void *owner) {
        tree->registerValue(data, owner);
        registerArray(&tree->thw[0], &data->thw[0], owner, {NORP});
        registerArray(&tree->dsMin[0], &data->dsMin[0], owner, {NORP});
        registerArray(&tree->P[0], &data->P[0], owner, {NORP});
        registerArray(&tree->D[0], &data->D[0], owner, {NORP});
    }

    void registerTree(__ParameterStopping *tree, ParameterStopping *data, const void *owner) {
        tree->registerValue(data, owner);
        tree->dsGap.registerValue(&data->dsGap, owner);
        tree->TMax.registerValue(&data->TMax, owner);
        tree->dsMax.registerValue(&data->dsMax, owner);
        tree->T.registerValue(&data->T, owner);
        tree->tSign.registerValue(&data->tSign, owner);
        tree->vStopped.registerValue(&data->vStopped, owner);
        tree->pedalDuringStanding.registerValue(&data->pedalDuringStanding, owner);
    }

    void registerTree(__ParameterLaneChange *tree, ParameterLaneChange *data, const void *owner) {
        tree->registerValue(data, owner);
        tree->bSafe.registerValue(&data->bSafe, owner);
        tree->aThreshold.registerValue(&data->aThreshold, owner);
        tree->politenessFactor.registerValue(&data->politenessFactor, owner);
        tree->time.registerValue(&data->time, owner);
    }

    void registerTree(__Input *tree, Input *data, const void *owner) {
        tree->registerValue(data, owner);
        registerTree(&tree->vehicle, &data->vehicle, owner);
        registerTree(&tree->horizon, &data->horizon, owner);
        registerStructArray(&tree->signals[0], &data->signals[0], owner, {NOS});
        registerStructArray(&tree->lanes[0], &data->lanes[0], owner, {NOL});
        registerStructArray(&tree->targets[0], &data->targets[0], owner, {NOT});
    }

    void registerTree(__State *tree, State *data, const void *owner) {
        tree->registerValue(data, owner);
        tree->simulationTime.registerValue(&data->simulationTime, owner);
        registerTree(&tree->decisions, &data->decisions, owner);
        registerTree(&tree->conscious, &data->conscious, owner);
        registerTree(&tree->subconscious, &data->subconscious, owner);
        registerArray(&tree->aux[0], &data->aux[0], owner, {NOA});
    }

    void registerTree(__Memory *tree, Memory *data, const void *owner) {
        tree->registerValue(data, owner);
        registerTree(&tree->vehicle, &data->vehicle, owner);
        tree->velocity.registerValue(&data->velocity, owner);
        registerTree(&tree->lateral, &data->lateral, owner);
        registerTree(&tree->laneChange, &data->laneChange, owner);
    }

    void registerTree(__Parameters *tree, Parameters *data, const void *owner) {
        tree->registerValue(data, owner);
        registerTree(&tree->vehicle, &data->vehicle, owner);
        registerTree(&tree->laneChange, &data->laneChange, owner);
        registerTree(&tree->stop, &data->stop, owner);
        registerTree(&tree->velocity, &data->velocity, owner);
        registerTree(&tree->follow, &data->follow, owner);
        registerTree(&tree->steering, &data->steering, owner);
    }



} // namespace

