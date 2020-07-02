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
// Created by Jens Klimke on 07.04.20.
// Contributors:
//


#ifndef SIMDRIVER_DISTANCETIMEINTERVAL_H
#define SIMDRIVER_DISTANCETIMEINTERVAL_H

#include "model_collection.h"
#include <cmath>


namespace agent_model {

    class DistanceTimeInterval {

    protected:

        double _actualPosition = 0.0; //!< The actual position
        double _actualTime = 0.0;     //!< The actual time

        double _startTime = INFINITY; //!< The start time
        double _endTime = INFINITY;   //!< The end time

        double _startPosition = INFINITY; //!< The start position
        double _endPosition = INFINITY;   //!< The end position

        double _scale = 1.0; //!< The factor to be scaled
        double _delta = 1.0; //!< The power to calculate the scale


    public:


        /**
         * Sets the power of the scale
         * @param delta Power of the scale
         */
        void setDelta(double delta) {

            _delta = delta;

        }


        /**
         * Returns a flag whether the interval is set or not
         * @return Flag
         */
        bool isSet() const {

            return !std::isinf(_startTime) || !std::isinf(_startPosition);

        }


        /**
         * Sets the scale for the factor
         * @param scale Scale
         */
        void setScale(double scale) {

            _scale = scale;

        }


        /**
         * Returns the scale parameter
         * @return The scale parameter
         */
        double getScale() const {

            return _scale;

        }


        /**
         * Update the actual state
         * @param position Actual position
         * @param time Actual time
         */
        void update(double position, double time) {

            _actualPosition = position;
            _actualTime = time;

        }


        /**
         * Sets the desired time interval
         * @param timeInterval Time interval
         */
        void setTimeInterval(double timeInterval) {

            if(std::isinf(timeInterval)) {

                // set start and end time to inf
                _startTime = INFINITY;
                _endTime = INFINITY;

                return;

            }

            _startTime = _actualTime;
            _endTime = _actualTime + timeInterval;

        }


        /**
         * Sets the desired end position of the interval
         * @param endPosition End position of the interval
         */
        void setEndPosition(double endPosition) {

            if(std::isinf(endPosition)) {

                // set start and end time to inf
                _startPosition = INFINITY;
                _endPosition = INFINITY;

                return;

            }

            _startPosition = _actualPosition;
            _endPosition = endPosition;

        }


        /**
         * Resets the set interval
         * Does not reset the actual position and time
         */
        void reset() {

            setTimeInterval(INFINITY);
            setEndPosition(INFINITY);

        }


        /**
         * Returns the normalized factor
         * @return The normalized factor
         */
        double getFactor() const {

            // if not set, return 0
            if(!isSet())
                return 0.0;

            // calculate factors
//            double ft = std::isinf(_startTime) ? 0.0 : agent_model::scale(_actualTime, _endTime, _startTime, 0.5);
//            double fs = std::isinf(_startPosition) ? 0.0 : agent_model::scale(_actualPosition, _endPosition, _startPosition, 0.5);

            double ft = std::isinf(_startTime) ? 0.0 : agent_model::scale(_actualTime, _endTime, _startTime, _delta);
            double fs = std::isinf(_startPosition) ? 0.0 : agent_model::scale(_actualPosition, _endPosition, _startPosition, _delta);

            // maximum
            return (std::max)(ft, fs);

        }


        /**
         * Returns the scaled factor of the interval
         * @return The scaled factor
         */
        double getScaledFactor() const {

            return getFactor() * _scale;

        }

    };

} // namespace

#endif //SIMDRIVER_DISTANCETIMEINTERVAL_H
