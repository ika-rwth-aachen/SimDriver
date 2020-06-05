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
// InjectionInterface.h

#ifndef DATA_GENERATOR_REGISTRATION_INTERFACE_H
#define DATA_GENERATOR_REGISTRATION_INTERFACE_H

#include <string>
#include <map>
#include <vector>

/**
 * The InjectionInterface implements a injection for an injection object with a associated owner, handles the
 * index and provides a method to apply all values of a given owner.
 */
struct InjectionInterface {

public:

    static std::map<const void *, std::vector<InjectionInterface *>> _index; //!< The index of all injection owners

    /**
     * Runs the apply() method of all elements associated with the given owner
     * @param owner Owner of the element
     */
    static void applyAll(const void *owner);


    /**
     * Runs the reset() method of all elements associated with the given owner
     * @param owner Owner of the element
     */
    static void resetAll(const void *owner);


    /**
     * Removes the object associated with the given owner
     * @param owner Owner to be removed
     */
    static void remove(const void *owner);


protected:


    /**
     * Resets the injection
     */
    virtual void reset() = 0;


    /**
     * Applies the actual value to the base value
     */
    virtual void apply() = 0;


    /**
     * Registers an object in combination with the owner
     * @param owner Owner of the object
     */
    void registerInjection(const void *owner);

};


#endif // DATA_GENERATOR_REGISTRATION_INTERFACE_H