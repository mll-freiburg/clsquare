/*
clsquare - closed loop simulation system
Copyright (c) 2004, Neuroinformatics Group, Prof. Dr. Martin Riedmiller,
University of Osnabrueck

Adapted from a previous version of moutaincar visualisation by Ralf Schoknecht and Artur Merke

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
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

#include "udpsocket.h"
#include "graphic.h"

#ifndef _MOUNTAINCAR_GRAPHIC_H_
#define _MOUNTAINCAR_GRAPHIC_H_


  /** Realizes a CLSquare Graphic-module for a mountain car plant.
   * \author Martin Riedmiller
   * \ingroup GRAPHIC
   * \see MountainCar. */
class MountainCarGraphic : public Graphic {
 public:

  
  virtual bool notify(const double *plant_state, const double *observation, const double *reference_input, 
                      const double *action, const long cycle, const long episode, const float total_time, 
                      const float episode_time, const long total_num_of_cycles);
  
  /** Initializes visualization of the environment.
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

 protected:
  /** (De-)Activate the UDP-communication to frameview. */
  bool graphic_active; 

  /** The port for UDP-communication (localhost). */
  int port; 

  /** The host on which frameview is running. */
  char hostname[500];

  /** A UDP socket for the communication with frameview */
  UDPsocket sock;  
  
  /** Read options for visualisation of mountain car.
   * \param fname File, which contains configuration (options).
   * \return true, for success.
   *
   * Options are:
   * \li active: False for deactivating visualisation.
   * \li port: UDP-port for communication. */   
  bool read_options(const char * fname);
};



#endif

