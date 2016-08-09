# CLSquare: Closed Loop Simulation System

CLS<sup>2</sup> (pronounced: clsquare) provides a standardized framework for testing (not only) Reinforcement Learning controllers on a (growing) number of different plants.

## Features

- Easy combination of different plants and controllers by definition in a configuration file.
- Lean interfaces for plants and controllers for a quick migration to and from CLS2 from and to other software environments.
- Optional graphical interface and visualization.
- Many demos for easy starting.

## Plants

- Simulations
     - Cart Pole (single, double and parallel), underactuated pole
     - Maze
     - Mountain Car
- Hardware Interfaces
     - Kinova Jaco
     - Robotis Bioloid

## Controllers

- Various static standard controllers (linear, PID…)
- Table-based Q Reinforcement Learning
- Neural Q and policy controllers
- Gaussian Processes
- Kernel-based batch controller (Ormoneit and Sen, 2002)
- Fitted Q Iteration with Extra Trees (Ernst, Geurts and Wehenkel, 2005)

## Supported Platforms

The software is developed and tested under:

- Ubuntu Linux 12.04
- Mac OSX 10.8

Basic requirements:

- a C++ compiler (build-essential for Ubuntu, xcode for OSX)
- CMake 2.6.3 (2.8.2 is required for libgp)

Each module that has special requirements will report them during installation if they are missing.

## Installation

- Get a fresh copy of CLS2
- Unpack the source code

            tar xzf clsquare.tgz
- Compile the program

            mkdir build
            cd build
            cmake ..
            make install

## Documentation

The most recent manual is included in the source package. 
In addition, documentation for the available modules can be generated using Doxygen by typing make doc in the program's build folder.

## People

Researchers contributing to this project:

- Prof. Dr. Martin Riedmiller
- Manuel Blum
- Thomas Lampe
- Dr. Roland Hafner
- Dr. Sascha Lange
- Dr. Stephan Timmer

## License

Copyright (c) 2016, University of Freiburg  
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the FreeBSD Project.

