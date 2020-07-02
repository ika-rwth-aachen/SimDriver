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
// Created by Jens Klimke on 2020-04-06.
// Contributors:
//


#ifndef SIMDRIVER_STOPHORIZON_H
#define SIMDRIVER_STOPHORIZON_H

#include <map>
#include <cmath>
#include <limits>

#ifndef EPS_TIME
#define EPS_TIME 1e-6
#endif

#ifndef EPS_DISTANCE
#define EPS_DISTANCE 1e-9
#endif

namespace agent_model {


    class StopHorizon {

    protected:

        constexpr static const double DELETE_AFTER_DISTANCE = 10.0; //!< Distance after which the stop point is deleted from the list

        struct _StopPoint {
            double s = INFINITY;
            double sStart = INFINITY;
            double timeStartStanding = INFINITY;
            double standingTime = INFINITY;
            bool passed = false;
        };

        double _sActual = 0.0;
        std::map<unsigned long, _StopPoint> _elements{};


    public:

        /** @brief a struct to store a stop point */
        struct StopPoint {
            unsigned long id;
            double ds;
            double interval;
        };


        /**
         * Inits the stop horizon
         * @param s Initial distance
         */
        void init(double s) {

            _sActual = s;
            _elements.clear();

        }


        /**
         * Adds a stop point to the list if it doesn't exist already
         * @param id ID of the stop
         * @param sStop Absolute position of the stop
         * @param standingTime The time the vehicle shall stand at the given stop (inf: until reset)
         * @return Flag to indicate if the stop point was added
         */
        bool addStopPoint(unsigned long id, double sStop, double standingTime) {

            // only add if not already added
            if(_elements.find(id) != _elements.end())
                return false;

            // only add when distance is large enough
            if(_sActual - sStop >= DELETE_AFTER_DISTANCE - EPS_DISTANCE)
                return false;

            // add to list
            _elements[id] = {sStop, _sActual, INFINITY, standingTime, false};

            return true;

        }



        /**
         * Marks the given stop as stopped
         * @param id ID of the stop
         * @param actualTime The actual simulation time
         * @return Returns a flag whether the time was set or not
         */
        bool stopped(unsigned long id, double actualTime) {

            // only set start time if not set before
            if(std::isinf(_elements.at(id).timeStartStanding)) {

                // set start time to actual time
                _elements.at(id).timeStartStanding = actualTime;

                // return success
                return true;

            }

            // not set
            return false;

        }


        /**
         * Updates the actual position
         * @param actualPosition Actual position
         * @param actualTime Actual simulation time
         */
        void update(double actualPosition, double actualTime) {

            _sActual = actualPosition;

            // iterate over elements
            for(auto &ke : _elements) {

                // get element
                auto &e = ke.second;

                // ignore passed stops
                if(e.passed || std::isinf(e.timeStartStanding))
                    continue;

                // set passed
                if(actualTime - e.timeStartStanding >= e.standingTime - EPS_TIME)
                    e.passed = true;

            }

            // clean up
            auto it = _elements.cbegin();
            while(it != _elements.cend()) {

                // delete elements after passed and distance large
                if(it->second.passed && _sActual - it->second.s >= DELETE_AFTER_DISTANCE - EPS_DISTANCE)
                    it = _elements.erase(it);
                else
                    it++;

            }


        }


        /**
         * Returns the next stop point
         * @return
         */
        StopPoint getNextStop() {

            // init
            double dsMin = INFINITY;
            double interval = INFINITY;
            unsigned long id = (std::numeric_limits<unsigned long>::max)();

            // iterate over elements
            for(auto &ke : _elements) {

                // get element
                auto &e = ke.second;

                // save distance
                double ds = e.s - _sActual;

                // ignore
                if(e.passed || ds > dsMin)
                    continue;

                // save data
                dsMin = ds;
                id = ke.first;
                interval = e.s - e.sStart;

            }

            // return;
            return StopPoint{id, dsMin, interval};

        }


    };


}

#endif // SIMDRIVER_STOPHORIZON_H
