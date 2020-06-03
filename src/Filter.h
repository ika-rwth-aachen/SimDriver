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
// Created by Jens Klimke on 2020-04-04
// Contributors:
//


#ifndef SIMDRIVER_FILTER_H
#define SIMDRIVER_FILTER_H

#include <deque>
#include <vector>

namespace agent_model {


    /**
     * @brief A class to implement a mean filter
     */
    class Filter {

    protected:

        unsigned int n; //!< Number of elements
        unsigned int i; //!< Current element's index (circular buffer)

        std::vector<double> _elements; //!< Element container

    public:

        /**
         * Initializes the filter with its length
         * The length is equal to the number of elements of which the mean value is calculated
         * @param length Length of the filter
         */
        void init(unsigned int length) {

            // set length and index
            n = length;
            i = 0;

            // create vector
            _elements.clear();
            _elements.reserve(n);

        }


        /**
         * Returns the filtered mean value of the elements
         * @return Filtered mean value
         */
        double value() {

            // special case
            if(_elements.empty())
                return 0.0;

            // sum up
            auto sum = 0.0;
            for (auto &e : _elements)
                sum += e;

            // return average value
            return sum / (double) _elements.size();

        }


        /**
         * Adds the current value and returns the actual filtered value
         * @param v Value to be added
         * @return The mean value
         */
        double value(double v) {

            // add element
            if(_elements.size() < n)
                _elements.emplace_back(v);
            else
                _elements.at(i) = v;

            // increment i
            i = (i + 1) % n;

            // return mean value
            return value();

        }

    };


}


#endif // SIMDRIVER_FILTER_H
