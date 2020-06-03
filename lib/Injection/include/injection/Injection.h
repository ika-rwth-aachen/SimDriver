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
// Created by Jens Klimke on 2020-02-12.
// Contributors:
//
// Injection.h


#ifndef DATA_GENERATOR_REGISTRATION_H
#define DATA_GENERATOR_REGISTRATION_H

#include <memory>
#include "InjectionInterface.h"

/**
 * @brief Injection class used to wrap a regular value and enable the injection of the value
 * @tparam T The datatype of wrapped value
 */
template<typename T>
class Injection : public InjectionInterface {

public:

    T *_ptr; //!< The calculated actual value
    std::unique_ptr<T> _inj; //!< The injected value


    /**
     * Default constructor
     */
    Injection() = default;


    /**
     * Copy constructor
     * @param v The value to be injected
     */
    explicit Injection(const T &v) {

        this->operator=(v);

    }


    /**
     * Default destructor
     */
    virtual ~Injection() = default;


    /**
     * Creates the injection
     * @param pointer Pointer to the base value
     * @param owner Owner of the value
     */
    void registerValue(T *pointer, const void *owner) {

        _ptr = pointer;
        registerInjection(owner);

    }


    /**
     * Converts the object to the actual value
     * @return The actual value
     */
    explicit operator T() const {
        return *_ptr;
    }


    /**
     * Returns the point to the actual value
     * @return Pointer to the actual value
     */
    T *ptr() {
        return _ptr;
    }


    /**
     * Sets the actual value
     * @param value The value to be set
     */
    Injection &operator=(const T &value) {
        _inj.reset(new T(value));
        return *this;
    }


protected:


    void apply() override {

        *_ptr = _inj ? *_inj : *_ptr;

    }


    void reset() override {

        _inj.reset(nullptr);

    }


};

#endif // DATA_GENERATOR_REGISTRATION_H