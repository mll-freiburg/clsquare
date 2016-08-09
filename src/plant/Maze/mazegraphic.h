/*
clsquare - closed loop simulation system
Copyright (c) 2004, Neuroinformatics Group, Prof. Dr. Martin Riedmiller,
University of Osnabrueck

Adapted from a previous version of maze visualisation by Ralf Schoknecht and Artur Merke

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
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF 
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

#ifndef __MAZE_GRAPHIC_H_
#define __MAZE_GRAPHIC_H_

/** Defines the maximal buffer size. */
#define MAX_SIZE_BUFFER 8096
/** The constant PI. */
#define PI 3.141592654

#include "tcpsocket.h"
#include "mazedata.h"
#include "graphic.h"

  /** Realizes a CLSquare Graphic-module for a maze plant.
   * \author Stephan Timmer
   * \ingroup GRAPHIC
   * \see Maze MazeData Cell. */
class MazeGraphic : public Graphic { 
 public:
  /** Updates visualization of the maze
   * \param state current state. 
   * \param observation observation of current state. 
   * \param referece_input reference_input of plant
   * \param action executed action in current state.
   * \param reward reward for executing action in current state
   * \param cycle_ctr control cycle in current episode.
   * \param episode current episode
   * \param total_time elapsed time since start of simulation loop.
   * \param episode_time elapsed time since start of current episode
   * \param total_num_of_cycles number of control cycles since start of simulation loop.
   * \return true, for success. */
  virtual bool notify(const double *plant_state, const double *observation, const double *reference_input, 
                      const double *action, const long cycle, const long episode, const float total_time, 
                      const float episode_time, const long total_num_of_cycles);
  
  /** Initializes visualization of the maze
   * \param state_dim dimension of plant state
   * \param observation_dim dimension of observation
   * \param reference_input_dim dimension of reference input
   * \param action_dim dimension of action space.
   * \param delta_t duration of one control cycle.
   * \param fname file, which contains conifiguration
   * \return true, for success. */
  virtual bool init(int _state_dim, int _observation_dim, int _action_dim, int _reference_input_dim, double _delta_t, const char *fname=0);

  /** Terminate graphics.
   * \return true for success */
  virtual bool deinit();
  
  /** Free memory for maze representation. */
  virtual ~MazeGraphic();
  
 protected:

  /** Representation of maze. */
  MazeData* maze;
  
  /** True, if player has already been built. False otherwise. */
  bool plBuilt;
  
  /** True, if maze has already been sent to a frameview. False otherwise.*/
  bool mazeSent;
  
  bool useObservation;
  
  /** (De-)Activate the TCP-communication to a frameview */
  bool graphic_active;

  /** Name of host on which frameview is running. */
  char hostname[500];

  /** The port for TCP-communication (on localhost) */
  int port;

  /** A TCP socket for communication with frameview */
  TCPsocket sock;
  
  /** Initially sends definition of maze to frameview.
   * This is done only once. */
  void sendMaze();
  
  /** Read options for visualisation of maze.
   * \param fname File, which contains configuration (options).
   * \return true, for success.    
   *
   * Options are:
   * \li active: False for deactivating visualisation.
   * \li port: TCP-port for communication. */  
  void read_options(const char* fname);
};

#endif
