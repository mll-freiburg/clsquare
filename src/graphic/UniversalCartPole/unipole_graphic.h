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

#include "udpsocket.h" // required for communication with frameview
#include "graphic.h"

#ifndef _UNIVERSAL_CARTPOLE_GRAPHIC_H_
#define _UNIVERSAL_CARTPOLE_GRAPHIC_H_

/** Realizes a CLSquare Graphic module for various pole plants.
  * It supports single, double and parallel cart poles, as well
  * as pure cart or pole systems.
  *
  * The plant state is assumed to be of dimensionality 2, 4 or
  * 6 and arranged in the order:
  * \li angle and angular velocity of the first pole (only if
  *     \b cart_only = \e false or state dim > 2)
  * \li angle and angular velocity of the second pole (only if
  *     state_dim = 6)
  * \li position and velocity of the cart (only if
  *     \b cart_only = \e true or state dim > 2)
  *
  * A cart-only visualization can be achieved by passing a
  * 2-dimensional state and setting the config parameter
  * \b cart_only to \e true.
  *
  * Additional parameters are:
  * \li \b hostname: name of host running frameview
  * \li \b port: UDP port for frameview
  * \li \b active: do not display any graphics; can be used
  *     with plants that automatically launch this graphic
  *     module
  * \li \b parallel: distinguishes between double and parallel
  *     pole systems
  * \li \b scale: scales the size of the graphics
  * \li \b pole_length, \b length_pole1: half length of the
  *     first pole in meters
  * 
  * \ingroup GRAPHIC
  * \author Martin Riedmiller */
class UniversalCartPoleGraphic : public Graphic {
 public:
  /** Updates visualization of the environment.
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
  virtual bool notify(const double *state, const double *observed_state,const double *reference_input,
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
  /** Show only cart? */
  bool cart_only;

  /** Dimension of state space */
  int x_dim;

  /** Dimension of action space */
  int u_dim;

   /** Scaling factor for graphical view. */
  double scale;

  /** Original pole length in cart-pole plant */
  double pole_length;

  /** Length of pole in graphical view. */
  double scaled_pole_len;

  /** Vertical Origin of first pole on cart. */
  double pole1_origin_y;

  /** (De-)Activate the UDP-communication to a Frameview2D */
  bool graphic_active; 

  /** Sets double pole to parallel mode. */
  bool parallel;

  /** The port for UDP-communication (on localhost) */
  int port;  
  
  /** Hostname for UDP-communication. */
  char hostname[500];

  /** A UDP socket for the communication with a Frameview2D */
  UDPsocket sock;   

  /** Read options for visualisation of cart-pole
   * \param fname File, which contains configuration (options).
   * \return true, for success.
   *
   * Options are:
   * \li active: False for deactivating visualisation.
   * \li port: UDP-port for communication. 
   * \li scale : scaling factor for graphical view.
   * \li pole_length : Length of first pole */   
  virtual bool read_options(const char * fname);
};



#endif
