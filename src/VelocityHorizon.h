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
// Created by Jens Klimke on 2020-04-02.
// Contributors:
//
// VelocityHorizon.h


#ifndef SIMDRIVER_VELOCITYHORIZON_H
#define SIMDRIVER_VELOCITYHORIZON_H

#include <cmath>
#include <deque>
#include "model_collection.h"


namespace agent_model {


    /** @brief A class to store the internal horizon */
    class VelocityHorizon {

    protected:

        /** @brief A class store a prediction point */
        struct PredictionPoint {
            size_t i;     //!< Reference index of the point
            double s;     //!< The longitudinal reference position of the point
            double ds;    //!< The actual distance to the point
            double vRule; //!< The planned velocity at the point
            double vCont; //!< The continuous velocity (e.g. curve speed)
            double sCont; //!< The continuous measure point
        };

        double _offset;
        double _vMax;

        std::deque <PredictionPoint> _elements{};



    public:


        /**
         * Initializes the points container
         * @param offset Position offset of the horizon
         * @param noOfElements Number of elements to be stored
         */
        void init(double offset, unsigned int noOfElements) {

            // set offset
            _offset = std::floor(offset);

            // reset elements
            _elements.clear();

            // create points
            for (size_t i = 0; i < noOfElements; ++i)
                _elements.emplace_back(newPoint(i));

        }


        /**
         * @brief Updates the horizon to the new reference position
         * Removes all elements with a distance smaller than zero, except of the first one smaller than zero
         * @param s New reference position
         */
        void update(double s) {

            size_t i0 = 0; // first element with positive distance

            for (auto &e : _elements) {

                // recalculate distance
                e.ds = e.s - s;

                // check distance and increment if <= 0
                if(e.ds <= 0.0)
                    i0++;
            }

            // get reference index of the last element
            size_t ib = _elements.back().i;

            // remove old element
            for (size_t i = 0; i + 1 < i0; ++i) {

                // new index
                size_t i1 = ib + i + 1;

                // remove from front, add to the back
                _elements.pop_front();
                _elements.emplace_back(newPoint(i1));

            }

        }


        /**
         * Returns the index of the start point of the interval in which the position is
         * @param s Position to be searched
         * @return Index of the interval
         */
        unsigned int getIndexBefore(double s) {

            double s0 = _elements.at(0).s;

            if(s <= s0)
                return 0.0;
            if(s >= _elements.back().s)
                return _elements.size() - 1;

            return (unsigned int) std::floor(s - s0);

        }


        /**
         * Returns the index of the end point of the interval in which the position is
         * @param s Position to be searched
         * @return Index of the interval
         */
        unsigned int getIndexAfter(double s) {

            double s0 = _elements.at(0).s;

            if(s <= s0)
                return 0.0;
            if(s >= _elements.back().s)
                return _elements.size() - 1;

            return (unsigned int) std::ceil(s - s0);

        }


        /**
         * Updates the maximum total velocity
         * @param v Velocity to be set
         */
        void setMaxVelocity(double v) {

            _vMax = v;

        }


        /**
         * Resets the set speed rules
         */
        void resetSpeedRule() {

            for(auto &e : _elements)
                e.vRule = INFINITY;

        }


        /**
         * Updates the speed in the given interval if the speed is smaller than the already set speed
         * @param s0 Start of the interval
         * @param s1 End of the interval
         * @param v Velocity to be set
         */
        void updateSpeedRuleInInterval(double s0, double s1, double v) {

            auto i0 = getIndexBefore(s0);
            auto i1 = getIndexAfter(s1);

            for (unsigned int i = i0; i <= i1; ++i) {

                // get element
                auto &e = _elements.at(i);

                // set speed if speed is smaller and point in interval
                if(e.vRule > v)
                    e.vRule = v;

            }

        }


        /**
         * Updates the continuous velocity profile at the given point
         * @param s Point to be set
         * @param v Velocity to be set
         */
        void updateContinuousPoint(double s, double v) {

            // get index before position
            auto i = getIndexAfter(s);
            auto &e = _elements.at(i);

            if(s > e.sCont) {

                e.sCont = s;
                e.vCont = v;

            }

        }



        /**
         * Calculates the mean speed within the given interval
         * @param s0 Start of the interval
         * @param s1 End of the interval
         * @param delta A factor shifting the influence over the interval
         * @return The mean value
         */
        double mean(double s0, double s1, double delta = 1.0) {

            // instantiate
            double v = 0.0;
            double vMin = INFINITY;
            double j = 0;

            // get indexes
            auto i0 = getIndexBefore(s0);
            auto i1 = getIndexAfter(s1);

            for (unsigned int i = i0; i <= i1; ++i) {

                // get speed and s
                auto v0 = (std::min)(vMin, getSpeedAt(i));
                auto s = _elements.at(i).s;

                // sum up with scaled factor
                auto f = agent_model::scale(s, s1, s0, delta);
                v += f * v0;

                // set minimum for future
                if(v0 < vMin)
                    vMin = v0;

                // divisor
                j += f;

            }

            return v / (double) j;

        }



    protected:


        /**
         * Returns the minimum of the speed at the given index
         * @param i Index
         * @return Minimum speed
         */
        double getSpeedAt(unsigned int i) {

            // get speed at index
            auto &e = _elements.at(i);
            return (std::min)((std::min)(e.vCont, e.vRule), _vMax);

        }


        /**
         * Creates a new point with the given index at the given position
         * @param i Index of the point
         * @return The new point
         */
        PredictionPoint newPoint(size_t i) {

            double s = _offset + (double) i;
            return PredictionPoint{i, s, INFINITY, INFINITY, INFINITY, s - 1.0};

        }


    };


}


#endif //SIMDRIVER_VELOCITYHORIZON_H
