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
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

#include "maze.h"
#include "valueparser.h"
#include <cstdlib>
#include "random.h"


#define STATE_DIM 2
#define ACTION_DIM 1


void Maze::deinit(){ 
  return ;
}

bool Maze::get_next_plant_state(const double *plant_state, const double *action, double *next_plant_state)
{
  
  int col = (int) plant_state[0]+.5;
  int row = (int) plant_state[1]+.5;

  Cell* act_cell = maze->get_cell(col,row);
 
  // Check for illegal action
  if (action[0] < 0 || action[0] >= NUM_MAZE_ACTIONS ) {
     std::cerr << "ERROR: " << __PRETTY_FUNCTION__ << ": illegal choice of action" << endl;    
     exit(1);
  }

  // Choose an action according to the probability treshold prob
  Cell* next_cell = 0;
  double rand = drand48();
  if (rand <= prob)      
    next_cell = act_cell->get_next_cell((int) action[0]);
  else {
    double step = (1.0 - prob) / (double) (NUM_MAZE_ACTIONS - 1);  
    double treshold = prob + step;
    for (int i = 0; i < NUM_MAZE_ACTIONS;i++) {
      if (i == (int) action[0])
        continue;
      if (rand < treshold) {	
        next_cell = act_cell->get_next_cell(i);	
        break;
      }
      treshold += step;
    }
  }

  next_plant_state[0] = next_cell->get_X();
  next_plant_state[1] = next_cell->get_Y();
  
  return true;
}

/* Not included in interface anymore. Need to use reward module.
bool Maze::is_terminal(const double *plant_state, double& terminal_rating) {
  terminal_rating = 0; // no terminal costs or rewards in maze.
  
  int col = (int) plant_state[0]+.5;
  int row = (int) plant_state[1]+.5;
  
  return maze->get_cell(col,row)->get_status() == GOAL;
}
*/

bool Maze::get_measurement(const double *state, double *measurement)
{
  if (jitter) {
    measurement[0] = state[0] + Util::urandom() * .98 - 0.49;
    measurement[1] = state[1] + Util::urandom() * .98 - 0.49;
  }
  else {
    measurement[0] = state[0];
    measurement[1] = state[1];
  }
  return true;
}




bool Maze::init(int& state_dim, int& measurement_dim, int& action_dim, double& delta_t, const char *fname, const char *chapter)
{
  prob = 1.0;
  maze = new MazeData((char*)"maze.def"); 
  // return values:
  state_dim = STATE_DIM;
  measurement_dim = state_dim; // default: observed_state is equal to state
  action_dim = ACTION_DIM;
  
  ValueParser vp(fname,chapter==0?"Plant":chapter);
  jitter = false;
  vp.get("jitter",jitter);
  
  
  return true;
}

bool Maze::check_initial_state(double *initial_state)
{
  Cell* cell = maze->get_cell((int) initial_state[0], (int) initial_state[1]);
  if (cell->get_status() == OBSTACLE) 
    return false;
  else
    return true;
}

Maze::~Maze()
{
  delete maze;
}


REGISTER_PLANT( Maze, "Maze-Benchmark" )











