// Copyright (c) 2019-2020 Jens Klimke <jens.klimke@rwth-aachen.de>
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
// Created by Jens Klimke on 2019-03-16
//

#ifndef SIMDRIVER_LOGGER_H
#define SIMDRIVER_LOGGER_H

#include <map>
#include <string>
#include <cmath>
#include <iostream>
#include <fstream>


class Logger {

    std::fstream _fstream;
    std::map<std::string, const double*> _values{};

    bool _hasContent = false;


public:


    /**
     * Creates the container
     * @param filename Filename of the log file
     */
    explicit Logger(const std::string &filename) {

        _fstream.open(filename, std::ios::out);
        _hasContent = false;

    }

    /**
     * Closes the container
     */
    ~Logger() {

        // close brackets
        if(_hasContent)
            (_fstream) << "\n]" << std::endl;

        // close fstream
        _fstream.close();

    }


    /**
     * Registers a double value to be added to the container
     * @param val Pointer to the value
     * @param key Key to be used in json
     */
    void registerValue(const std::string &key, const double *val) {

        if(key == "time")
            throw std::invalid_argument("time key word is reserved.");

        _values[key] = val;

    }


    /**
     * Writes the values to the stream
     * @param time Actual timestamp
     */
    void write(double time) {

        // save time and open object brackets
        (_fstream) << (_hasContent ? ",\n" : "[\n") << "\t" << R"({"time":)" << time << ",";

        // write data
        unsigned int i = 0;
        for(auto &p : _values) {

            // stream field name
            (_fstream) << (i++ == 0 ? "" : ",") << "\"" << p.first << "\":";

            // check for inf and nan
            if(std::isinf(*p.second) || std::isnan(*p.second))
                (_fstream) << "null";
            else
                (_fstream) << *p.second;

        }

        // close object brackets
        (_fstream) << "}";

        // save that data was already written
        _hasContent = true;

    }


};


#endif // SIMDRIVER_LOGGER_H
