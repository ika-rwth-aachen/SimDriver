![SimDriver CI](https://github.com/ika-rwth-aachen/SimDriver/workflows/SimDriver%20CI/badge.svg?branch=master)

# SimDriver

A responsive driver model for traffic simulations to create exact and closed-loop microscopic traffic scenarios.

## Motivation

There are several applications in research and development with the need for simulating traffic; e.g. development of intelligent transportation systems, road and traffic optimization, driver behavior studies, vehicle and vehicle component development, impact assessment for ADAS, just to name a few. Different applications desire different levels of details for the simulated elements of the traffic (road environment, vehicle, road participants) [[1]](https://en.wikipedia.org/wiki/Urban_traffic_modeling_and_analysis). The challenge of developing and validating Automated Driving Systems (ADS), however, is another example for the necessity of simulations [[2]](https://link.springer.com/chapter/10.1007/978-3-662-48847-8_21). Dividing the ADS into the sub-tasks _sense_, _plan_ and _act_, we can identify different requirements for the simulation. For the sensing task, sensor models and a detailed representation of the environment with material assignment might be necessary. For the act task, detailed vehicle dynamics and the component models are used as well as tire models and a detailed representation of the road surface. For the planning task of the ADS, the system needs to interpret and predict the _microscopic_ traffic situation to create a safe and goal-oriented driving strategy within the next seconds. To develop this functionality and prove its correct operation, simulations with the focus on relevant traffic scenarios are desired. Research projects like PEGASUS [[3]](https://www.pegasusprojekt.de/en/home), SET Level 4to5 [[4]](https://www.ika.rwth-aachen.de/en/research/projects/automated-driving/3126-set-level-4to5.html), VVMethoden [[5]](https://www.ika.rwth-aachen.de/de/forschung/projekte/automatisiertes-fahren/3119-vv-methoden.html) and many others put their efforts into this. 

Simulating traffic scenarios means, that you are able to depict the behavior of the relevant traffic environment, especially the behavior of other road users, which effect the decision and planning of the ADS. The PEGASUS approach is to generate trajectories [[6]](https://www.researchgate.net/publication/334978903_A_framework_for_definition_of_logical_scenarios_for_safety_assurance_of_automated_driving) for the challenging road participants (agents), forcing the ADS to react and/or restricting the ADS in the reaction. With this approach, an infinite number of scenarios can be generated exactly as desired, but the space of possible scenarios is still limited to pre-calculated, non-responsive behavior. Imagine you want to simulate an agent with a very specific microscopic behavior but also with a responsive behavior on the reaction of the vehicle under test (VuT) - a closed-loop agent model. A very simple example is the pushy driver (or tailgater). For an exemplary test with such a tailgater, the time headway (distance divided by velocity) shall be 0.5 seconds to the VuT and the time headway shall be kept during the scenario, also if the ADS decelerated or accelerates. With a pre-calculated trajectory this scenario cannot be generated, because of the unknown behavior of the ADS. 

This project provides a closed-loop agent model, implementing a set of capabilities (see below), which can be used to create traffic scenarios with dedicated KPIs on different levels of driving task (decision, guidance and stabilisation). These basic capabilities can be used to create specific scenarios by tuning the parameters. However, the basis is a specified normal behavior, enabling the model to just drive around, following others, making lane changes when desired and stop when conflicts occur. The behavior can be parameterized as desired in many details (see parameters and injection). The time headway for example can be directly set by a parameter in the model. The agent exactly controls the time headway after performing a smooth approaching maneuver from its cruising speed to the final following position. We call this idealized and plausible normal behavior.

The model does not claim to depict or even reproduce statistically _realistic_ human driving behavior. It is **not** validated in terms of performing maneuvers as the certain group of humans does (e.g. represented by a data set). However, we expect, that a realistic driving behavior can be depicted with this model by tuning the parameters to the data set (e.g. [this one](https://levelxdata.fka.de/en/)).

You will find detailed information about the scientific approach and the detailed structure in the publication mentioned below. The interface and possibilities of calibration and manipulation are described in the API section. 
 
## Capabilities and Features

In this documentation, the term _capability feature_ or _capabilities_ is used to describe, what the driver model is able to perform in terms of driving a vehicle - so the algorithmic part of the software. The term _software feature_ is used for the functions of the software to enable a user to use or to integrate the model into other software. The input interface for example is a software feature, while performing a lane change is a capability feature.

The driver model includes the following capabilities of basic driving:
- [x] Hold the vehicle in standstill during standing
- [x] Accelerate or decelerate the vehicle as desired
- [x] Reach and keep a desired velocity
- [x] Reach and keep a desired velocity in a defined distance
- [x] Stop the vehicle in a desired distance
- [x] Keep a desired longitudinal distance to a moving object 
- [x] Reach and keep a desired curvature
- [x] Follow desired track points
- [x] Keep a desired lateral offset to a track
- [x] Switch from one track to a parallel one smoothly

These basic and continuous capabilities can be used in discrete maneuvers and higher level capabilities. The following maneuver and capabilities are already implemented:

- [x] Follow other road participants in the traffic flow
- [x] Adapt the speed in curves
- [x] Adapt the speed according to speed limits
- [x] Decide and perform a lane change in traffic flow
- [x] Decide and perform a stopping maneuver at a stop sign

Of course there are plenty maneuvers and capabilities in addition, which are needed to drive on highways and in cities, which are not implemented yet. We plan to priorize the capabilities and implement it one by one. A few examples for planned capabilities:

- [ ] Dedicated behavior in stop&go traffic or with low speed in general
- [ ] Decide a lane change due to end of lane or conflict ahead
- [ ] Decide a lane change due to follow the route
- [ ] Stop at traffic lights
- [ ] Stop to give right of the way
- [ ] Slow down or stop for conflicts ahead
- [ ] Turning on intersections (can be done by the above capabilities)

The agent model should be seen as a driving framework for microscopic traffic participants to generate environmental traffic. Therefore a complex interface is created to:

* Create the regular **input** of the driver model, to describe the road structure, the other road participants, rules, etc.
* Set and change **parameters** to calibrate the agent specific regular behavior (e.g. maximum comfortable acceleration, accepted lateral acceleration in curves)
* Manipulate internal driver states to force maneuvers, driving errors or other irregular specific behavior. Theses states are called **guidance values** and have a physical and logical meaning in terms of driving and traffic. The manipulation is called injection. The injection concept and the purpose of it will be described in future publications.

## API

The core module of the agent model is a static library (``AgentModel`` class) with an ``init`` and a ``step`` method to be called before the simulation starts and in each simulation step respectively. The input, the parameters and also the states can be accessed (read and write) via pointers to the interface data structure. Therefore the methods ``getInput``, ``getState`` and ``getParameters`` are available in the class, which return the regarding pointers.

The model interface is defined in the Interface header file to separate this complex structure from the model implementation. The interface is code-generated. 

The raw library can be integrated straight forward into any C++ project. There are no dependencies at all except the std library. Additionally, we provide a CarMaker adapter that makes it possible to control the CarMaker ego vehicle and also target vehicles with the agent model. A detailed documentation of this feature can be found in `app/CarMaker`.

## Installation

Here we demonstrate how you build the library for static linking with C++ projects.

Required toolchain:
- a c++ compiler and make
- cmake (tested with version 3.16.1 and 3.10.2)
- git (when building with tests etc. to load submodules)
- Googletest (if test shall be build)
- nodejs and npm (if interface generation is desired, usually you don't need that)

Download:
- `> git clone --recursive <url> <dir>`

Building:
- `> cd <path-to-project> && mkdir build`
- `> cd build && cmake -G "Unix Makefiles" ..`
- `> make`

To build and run the tests, simply add the regarding parameter

- `> cd <path-to-project> && mkdir build`
- `> cd build && cmake -G "Unix Makefiles" -DBUILD_TESTS=ON ..`
- `> make && make test`

If these steps are performed correctly the agent model shall be generated in the build folder. To add the model to you cmake project, just use `add_submodule(...)` and include the project.
  
Build options:
- `-DBUILD_TESTS=(ON|OFF)`: Building tests executables and gtest
- `-DBUILD_FOR_COVERAGE=(ON|OFF)`: Building with code coverage option (Apple and Unix only)
- `-DBUILD_WITH_INJECTION=(ON|OFF)`: Activates or deactivates the injection feature (see section Usage and API)
- `BUILD_GTEST=(ON|OFF)`: If you do not have Googletest installed, clone it to the root folder and set this option to ON (see Dockerfile)

Building (including tests) is tested with:
- MacOS Catalina with Apple clang 11.0.0 
- Ubuntu 18.04.1 with g++ 7.4.0 (Docker)

## Authors
* Jens Klimke (jens.klimke@rwth-aachen.de)
* Daniel Becker (daniel.becker@ika.rwth-aachen.de)

Supervisor:
* Univ.-Prof. Dr.-Ing. Lutz Eckstein

We are very thankful for help. If you would like to join the developer team, please contact one of the main authors.

## Publications

       @inproceedings{simDriverAgentModel,
                title={System Design of a Driver Model for Closed-Loop Simulations of ADS-relevant Traffic Scenarios},
                author={Klimke, Jens and Becker, Daniel and Eckstein, Lutz},
                booktitle={29th Aachen Colloquium Sustainable Mobility 2020},
                keywords={driver model, simulation, automated driving, closed-loop, scenarios}
                venue={29th Aachen Colloquium Sustainable Mobility 2020, October 5th to 7th, 2020},
                location={Aachen, Germany},
                pages={to appear}
                year={2020}

## Help and support

If you have any problems, please don't hesitate to contact Daniel or Jens (see [authors](#Authors)). If you are a commercial user and need professional support, please contact our engineering partner fka Aachen GmbH [info@fka.de](mailto:info@fka.de).

## For contributers and forkers

To edit or regenerate the interface and correlated files of the driver model (AgentModelInterface[Injection].(h|cpp)), developers should use the DataGenerator tool found als submodule in the lib/ folder. If you create the cmake project with -DCREATE_INTERFACE_GENERATOR_TARGET=ON, a target is created, which generates the files based on the configuration in resources/interface_generator. To change the interface, please edit the json and csv file in this folder and regenerate the source code.

Trouble shooting: If the generation doesn't work due to missing nodejs packages, do a `cd lib/DataGenerator && npm install`. nodejs and npm need to be installed (see also Dockerfile). Then, try to build the generate_interface target again.

Why do we do it like this? The interface is very complex. A csv table of all data fields is much easier to understand than a header file. Additionally, it is quite difficult to keep the corresponding injection code synchronous to the actual interface. The generator does all of that for you.

## TODOs:
* GROUP Documentation:
    * ~~Add this README to doxygen main page~~~
    * ~~Listing in README with actual values (doxygen cannot parse it correctly)~~
    * Publish doxygen (on github?)
    * Move API to a separate file and add to doxygen.
* GROUP Dockerfile:
    * ~~Add doxygen to Dockerfile~~
* GROUP Model collection:
    * Remove the exceptions in the model collection, the invalid values should lead to defined results. If values are not valid in the simulation or model, they should be caught before the model collection function is called.
* GROUP Interface:
    * ~~Add class descriptions to interface generator~~
    * ~~Add generator library and config to project. Also create a custom target to cmake to generate the interface~~
    * Rename capabilities consistantly with or without ...ing (e.g. stopping, following, ...)
* GROUP Test environment:
    * Plot/Log: log each test in separate files (menu from old index.html)
    * ~~Create a smooth vehicle model~~ 
    * Limit acceleration of the vehicle model.
    * Write a generic checker (in GenericTest) for steps and oscillations in signals (e.g. acceleration, pedal, ...)
    * Write a test for injecting the interface (especially arrays, multi-dimensional arrays, ...)
* GROUP Examples (as tests and as CarMaker Demo):
    * Random traffic (traffic)
    * Tailgater (ego)
    * Cut-in scenario (ego)
    * Cut-out scenario (ego)
        * Extended: Evasion blocked by another agent
    * Follow a trajectory (ego) / Follow another object on a plane (no lane bounds, ego)
* GROUP Structure:
    * Move vehicle model and simulation from app to lib
* GROUP Last checks:
    * check the header directives to be consistent
    * check the CMakeLists for unnecessary code
    * Perform clang checker -> eliminate warnings
    * Eliminate TODOs in code
    * All header comments correct (dates, copyright, filenames, etc.)
    
## References
* [1] https://en.wikipedia.org/wiki/Urban_traffic_modeling_and_analysis
* [2] https://link.springer.com/chapter/10.1007/978-3-662-48847-8_21
* [3] https://www.pegasusprojekt.de/en/home
* [4] https://www.ika.rwth-aachen.de/en/research/projects/automated-driving/3126-set-level-4to5.html
* [5] https://www.ika.rwth-aachen.de/de/forschung/projekte/automatisiertes-fahren/3119-vv-methoden.html
* [6] Weber, Hendrik & Bock, Julian & Klimke, Jens & Rösener, Christian & Hiller, Johannes & Krajewski, Robert & Zlocki, Adrian & Eckstein, Lutz. (2019). _A framework for definition of logical scenarios for safety assurance of automated driving._ Traffic Injury Prevention. 20. S65-S70. 10.1080/15389588.2019.1630827. 