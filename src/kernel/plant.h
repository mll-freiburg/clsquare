/*
clsquare - closed loop simulation system
Copyright (c) 2004, Neuroinformatics Group, Prof. Dr. Martin Riedmiller,
University of Osnabrueck
Copyright (c) 2011, Machine Learning Lab, Prof. Dr. Martin Riedmiller,
University of Freiburg

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
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE. 
*/

#ifndef _plant_h_
#define _plant_h_

#include "global.h"

/** @defgroup PLANT Plant Modules
  * The plant module implements the behaviour of the system under investigation, how it behaves over time and the influence of actions applied to it.
  * Its main functionality is to compute time discrete dynamic: \f$\plantstate_{t+1} = f(\plantstate_{t},\action_{t})\f$ (potentially dependent on t and subject to noise).
  * Examples:
  * \li simple rule based system behaviour (e.g. CatAndMouse grid example)
  * \li numerical simulations of dynamic systems (e.g. Acrobot example)
  * \li external applications by means of communication (e.g. TCP communication to a Matlab simulation)
  * \li real systems by communication with hardware
  *
  * Main functionality is the function
  * virtual bool Plant::get_next_plant_state(const double *current_plant_state, const double *current_action, double *next_plant_state);
  * that defines the dynamic of the plant.
  */
/** @defgroup HARDWARE Hardware Plants
  * These Plant modules require robots or other hardware systems.
  * Some may include a simulation and can be used without the hardware,
  * in which case they should be listed in the Simulated Plant group.
  * \warning When designing a hardware plant, please ensure that the
  * function notify_command_string() covers the argument
  * "plant_cmd pause" and stops the physical system. Failure to do so
  * may result in damage as the control loop stops but the hardware
  * continues moving.
  *
  * @ingroup PLANT
  */
/** @defgroup SIMULATION Simulated Plants
  * These Plant modules include a simulated system and do not require any
  * special hardware.
  * @ingroup PLANT
  */

/** Base class for all plant modules in CLSquare. */
class Plant {
public:
  /** Computes the next state of the plant, given a current state and an action (state transition).  
   * \param current_plant_state current state of plant.
   * \param current_action executed action in current state.
   * \param current_external_signal the reference input vector
   * \param next_plant_state resulting state after executing action.
   * \return true to continue, false, if break of episode is requested */
  virtual bool get_next_plant_state(const double *current_plant_state, const double *current_action, const double *current_external_signal, double *next_plant_state);
  /** Simplyfied version for convenience, if no external_signal is used.
    * \attention do not reimplement both functions in your plant!
    */
  virtual bool get_next_plant_state(const double *current_plant_state, const double *current_action, double *next_plant_state);


  /** Computes a measurement of the plant_state. 
   * \param plant_state     current state
   * \param external_signal an external input for non controllable variables like reference values
   * \param observed_state  write observation of current state in this array
   * \return true for success */
  virtual bool get_measurement(const double *plant_state, const double *external_signal, double *measurement);
  virtual bool get_measurement(const double *plant_state, double *measurement);

  /** Checks, if plant agrees on initial state. If not, the plant module can override the initial state
   *  or reject the initial state by returning false. 
   * \param initial_state initial state
   * \returns true, if plant agrees on initial state or overrides the initial state */
  virtual bool check_initial_state(double *initial_plant_state);

  /** Checks, if plant agrees on reference input. If not, the plant can override or reject the reference input
    * by returning false.
    * \attention this function is called in every time step
    * \attention if plant changes reference input, the plant has to care for further time steps when the same reference input is applied.
    */
  virtual bool check_external_signal(double* /*reference_inp*/){return true;}

  /** Initialize plant.
   * Implement one of the following virtual bool init(...) functions for your plant.
   * For convenience you can use the function that meets your requirements most.
   * Other params are set by default as defined below.
   * \param plant_state_dim plant returns dimension of state vector for plant state.
   * \param measurenebt_dim plant returns dimension of observation vector for controller.
   * \param action_dim plant returns dimension of action space. 
   * \param delta_t plant returns duration of one control cycle in seconds.
   * \param fname File, which contains configuration of plant module
   * \return true, for success. */
  virtual bool init(int& plant_state_dim, int& measurement_dim, int& action_dim, int& external_signal_dim, double& delta_t, const char* fname=0, const char* chapter=0);
  virtual bool init(int& plant_state_dim, int& measurement_dim, int& action_dim, double& delta_t, const char *fname=0, const char* chapter=0);
  virtual bool init(int& plant_state_dim, int& action_dim, double& delta_t, const char *fname=0, const char* chapter=0);
  
  virtual void deinit(){;}
  
  /** Notifies that an episode has been started. */
  virtual void notify_episode_starts(){return;}
  
  /** Notifies that an episode has been stopped. */
  virtual void notify_episode_stops(){return;}

  /** Gets the name of the graphics module to use if none has been specified explicitly. */
  virtual const std::string get_default_graphics(){return "DefaultGraphic";};

  /** Notifies that a command via pipe has arrived. */
  virtual void notify_command_string(const char* buf);

  virtual void notify_suspend_for_aux_call_cmd() {return;}
  virtual void notify_return_from_aux_call_cmd() {return ;}

  /** virtual destructor is necessary since methods declared virtual */
  virtual ~Plant() {}

  /** internal functionality **/
  bool init_main(int& plant_state_dim, int& measurement_dim, int& action_dim, int& external_signal_dim, double& delta_t, const char* fname=0, const char* chapter=0);

protected:
  int __plant_state_dim;
  int __measurement_dim;
  int __action_dim;
  double __delta_t;
};

#ifdef CLSQUARE
#include "registry.h"
#else
#define REGISTER_PLANT(classname, desc)
#endif

#endif
