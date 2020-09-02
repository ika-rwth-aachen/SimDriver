// Copyright (c) 2019 Institute for Automotive Engineering (ika), RWTH Aachen University. All rights reserved.
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
// Created by Jens Klimke on 2019-07-22.
// Contributors:
//
// PlotLogger.h
//

#ifndef SIMCORE_PLOTLOGGER_H
#define SIMCORE_PLOTLOGGER_H

#include <sstream>
#include <fstream>
#include <utility>
#include <map>
#include <vector>

class PlotLogger {

    struct Trace {
        std::string name;
        std::string color{"black"};
        unsigned int lineWidth{1};
        std::vector<double> x{};
        std::vector<double> y{};
        std::string xRef{};
        std::string yRef{};
    };

    struct Figure {
        std::string title;
        std::string xLabel;
        std::string yLabel;
        bool axisEqual{false};
        std::vector<Trace> traces{};
    };

    std::string _title;
    std::string _filename;
    std::fstream _file;
    std::vector<Figure*> _figures{};
    std::map<std::string, Figure*> _figureIndex;
    std::vector<std::string> _fields{};
    std::string _dataFile{};
    std::vector<double*> _values{};

    bool _locked = false;
    bool _firstRow = true;


public:

    PlotLogger() = default;

    ~PlotLogger() override {

        // delete all figures
        for(auto f : _figures)
            delete f;

    }

    void create(std::string fname, std::string title) {

        _filename = std::move(fname);
        _title = std::move(title);

    }

    Figure *createFigure(const std::string& label) {

        // create figure
        _figures.emplace_back(new Figure);

        // get figure and add to index
        Figure *fig = _figures.back();
        _figureIndex[label] = fig;

        return fig;

    }

    void addFigure(const std::string& label, const std::string& title, const std::string &xLabel,
                   const std::string &yLabel, const std::string& xValues, const std::string& yValues, bool axisEqual = false) {

        addFigure(label, title, xLabel, yLabel, xValues, {{yValues, yValues}}, axisEqual);

    }

    void addFigure(const std::string& label, const std::string& title, const std::string &xLabel, const std::string &yLabel,
                   const std::string& xValues, const std::vector<std::pair<std::string, std::string>>& yValues, bool axisEqual = false) {

        // create and get figure
        auto fig = createFigure(label);

        // set attributes
        fig->title = title;
        fig->xLabel = xLabel;
        fig->yLabel = yLabel;
        fig->axisEqual = axisEqual;
        fig->traces.clear();

        // add traces
        for(const auto &v : yValues)
            trace(label, v.first, xValues, v.second, "auto", 2);

    }


    Trace *trace(const std::string &label, const std::string &name, const std::string &xValues,
                 const std::string &yValues, const std::string &color, unsigned int lineWidth) {

        // add trace
        _figureIndex.at(label)->traces.emplace_back();

        // get trace and set attributes
        auto tr = &_figureIndex.at(label)->traces.back();
        tr->name      = name;
        tr->xRef      = xValues;
        tr->yRef      = yValues;
        tr->color     = color;
        tr->lineWidth = lineWidth;

        return tr;

    }


    Trace *trace(const std::string& label, const std::string &name, const std::vector<double> &x,
                 const std::vector<double> &y, const std::string &color, unsigned int lineWidth) {

        // set trace
        auto tr = trace(label, name, "", "", color, lineWidth);

        // set data
        tr->x = x;
        tr->y = y;

        return tr;

    }


    void setDataFile(const std::string &dataFileName) {

        if(_locked)
            throw std::runtime_error("The file has been locked.");

        _dataFile = dataFileName;
        _fields.clear();
        _values.clear();

    }


    void defineDataset(std::vector<std::string>&& fieldnames, std::vector<double*>&& values) {

        if(_locked)
            throw std::runtime_error("The file has been locked.");

        _fields = std::move(fieldnames);
        _values = std::move(values);
        _dataFile.clear();

    }


    /**
     * Writes the current data
     */
    void writeData() {

        // write header if not written yet
        if(!_locked)
            writeHeader();

        // write row
        _file << (_firstRow ? "" : ",") << "{";
        for(size_t i = 0; i < _fields.size(); ++i)
            _file << (i == 0 ? "" : ",") << "\"" << _fields.at(i).c_str() << "\":" << *_values.at(i);
        _file << "}";

        _firstRow = false;

    }


    /**
     * Creates and opens the log file
     */
    void openFile() {

        // create and open file
        _file = std::fstream(_filename.c_str(), std::ios::out);

        // write header
        writeHeader();

    }


    /**
     * Closes the file
     */
    void closeFile() {

        // write header if not written yet
        if(!_locked)
            writeHeader();

        // check if data file is defined
        if(_dataFile.empty())
            _file << "]}";

        _file.close();

    }



protected:


    /**
     * Writes the file header
     */
    void writeHeader() {

        _firstRow = true;
        _locked = true;

        _file << R"({"title":")" << _title.c_str() << R"(",)"
                                                      R"("plots":[)";

        bool first = true;
        for(auto const &p : _figures) {

            std::string eq = "";
            if(p->axisEqual)
                eq = ",\"scaleanchor\":\"x\", \"scaleratio\":1";

            _file << (first ? "" : ",") << std::endl << R"( {"layout":{"title":")" << p->title
                  << R"(","xaxis":{"title":")" << p->xLabel << R"(","showgrid":true,"zeroline":true})"
                  << R"(,"yaxis":{"title":")" << p->yLabel << R"(","showgrid":true,"zeroline":true)" << eq << "}"
                  << R"(},"traces":[)";

            for(size_t i = 0; i < p->traces.size(); ++i) {

                // get trace
                auto tr = &p->traces.at(i);

                // create string stream for x and y data
                std::stringstream xData;
                std::stringstream yData;

                // store x values
                for(unsigned int j = 0; j < tr->x.size(); ++j)
                    xData << (j == 0 ? "[" : ",") << tr->x.at(j);

                // store y data
                for(unsigned int j = 0; j < tr->y.size(); ++j)
                    yData << (j == 0 ? "[" : ",") << tr->y.at(j);

                // set closing bracket
                xData << "]";
                yData << "]";

                // set xRef, yRef
                std::stringstream xRef, yRef;
                xRef << "\"" << tr->xRef << "\"";
                yRef << "\"" << tr->yRef << "\"";

                // store to file
                _file << (i == 0 ? "" : ",")
                      << std::endl
                      << R"({"name":")" << tr->name << "\""
                      << R"(,"x":)" << (tr->x.empty() ? xRef.str() : xData.str())
                      << R"(,"y":)" << (tr->y.empty() ? yRef.str() : yData.str())
                      << R"(,"type":"scatter"})";

            }

            _file << "]}";

            // TODO: equal axis
            // TODO: line width
            // TODO: color
            // TODO: line groups

            first = false;

        }

        if(!_dataFile.empty()) {
            _file << std::endl << R"(],"datafile":")" << _dataFile << "\"}";
            _file.close();
        } else
            _file << std::endl << R"(],"dataset":[)";

    }


};


#endif // SIMCORE_PLOTLOGGER_H
