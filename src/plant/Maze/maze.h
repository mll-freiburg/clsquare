/*
clsquare - closed loop simulation system
Copyright (c) 2004, Neuroinformatics Group, Prof. Dr. Martin Riedmiller,
University of Osnabrueck

Author: Stephan Timmer

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the <ORGANIZATION> nor the names of its
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

#ifndef _MAZE_H_
#define _MAZE_H_

#include "valueparser.h"
#include "mazedata.h"
#include "plant.h"
#include <iostream>
#include <fstream>


/** Realizes a maze plant for CLSquare.
 * Defines all methods necessary for a CLSquare plant. 
 * \see MazeData Cell. 
 * \author Stephan Timmer
 */
class Maze : public Plant{
 public:
  /** Computes the next state of the environment, given a current state and an action (state transition).  
   * \param state current state of plant
   * \param action executed action in current state
   * \param reference_input an external input for non controllable variables like reference values
   * \param next_state resulting state after executing action
   * \param reward reward for state transition  
   * \returns true, if next state is not a terminal state. */   
  virtual bool get_next_plant_state(const double *plant_state, const double *action, double *next_plant_state);
 
  /** Initializes Maze.
   * \param state_dim plant returns dimension of state space (state vector)
   * \param observation_dim plant returns dimension of observation space (observation vector) 
   * \param action_dim plant returns dimension of action space.   
   * \param delta_t plant returns duration of one control cycle [s]
   * \param fname file, which contains configuration of plant module
   * \return true, for success. */ 
  virtual bool init(int& state_dim, int& observation_dim, int& action_dim, double& delta_t, const char *fname=0, const char *chapter=0); 

  /** Sets an initial state.
   * Some starting states may be invalid, because an episode cannot start in an obstacle cell.
   * \param state Initial state
   * \param target_state Not necessary for maze plant.
   * \return true, if state is a valid starting state. */
  bool check_initial_state(double *initial_state);

  /** Not necessary for maze plant. */
  void deinit(); 
  
  /** calculates an observation from an internal plant state. This 
   method realizes (optional) noisy observations given exact state
   transitions. */
  bool get_measurement(const double *state, double *measurement);

  /** determins whether or not the given plant_state is a terminal
   state and optionally returns terminal costs. In the Maze plant,
   terminal costs area always zero. */
  //bool is_terminal(const double *plant_state, double& terminal_rating);

 protected:

  /** The maze (inner representation). */
  MazeData* maze;
  bool jitter;

  /** Free memory of maze representation. */
  ~Maze();

  /** Probability for performing the desired action. 
   *  All other actions are taken with probability (1-prob)/NUM_ACTIONS */
  double prob;
   
};


#endif

