/*
 * Copyright 2020 Jens Klimke
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * author: Jens Klimke
 * date: 2020-02-16
 */

#include <Injection/Interface.h>

namespace sim_driver::injection {

    std::map<const void *, std::vector<Interface *>> Interface::_index{};


    void Interface::applyAll(const void *owner) {

        // iterate over elements and apply
        for (auto &e : _index.at(owner))
            e->apply();

    }


    void Interface::resetAll(const void *owner) {

        // iterate over elements and apply
        for (auto e : _index.at(owner))
            e->reset();

    }


    void Interface::registerInjection(const void *owner) {

        // create owner index
        if (_index.find(owner) == _index.end())
            _index[owner] = std::vector<Interface *>();

        // add this
        _index.at(owner).push_back(this);

    }


    void Interface::remove(const void *owner) {

        // delete owner
        if (_index.find(owner) != _index.end())
            _index.erase(owner);

    }

}