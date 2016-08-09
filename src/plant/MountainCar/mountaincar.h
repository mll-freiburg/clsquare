/*
clsquare - closed loop simulation system
Copyright (c) 2004, Neuroinformatics Group, Prof. Dr. Martin Riedmiller,
University of Osnabrueck

mountaincar.c is adapted from a previous implementation by Ralf Schoknecht and Artur Merke

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
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
*/


 /*! \class MountainCar
 *  \brief Implementation for a MountainCar plant.
 *
 *  \section Space Description
 *  \c plant_state: 2 dimesnional
 *  \li {0} position of car
 *  \li {1} velocity of car
 *
 *  \c observations: 2 dimensional
 *  same as plant_state
 *
 *  \c actions: 1 dimensional
 *  \li {0} force on car
 *  
 *  \section Configurable Parameters
 *  Following parameters are read in section [Plant] of the configuration file:
 *  \li \c delta_t  (double) time for one timestep to be integrated in [s]  (0.05)
 *  \li \c pos_min  (double) minimal position, run will be ended if positions smaller accure [m]  (-1.0)
 *  \li \c pos_max  (double) maximal position, run will be ended if positions bigger accure [m]  (0.9)
 *  \li \c u_max    (double) maximal allowed action magnitude, allowed action range: [-u_max,u_max] (5)
 *
 * @ingroup PLANT
 * @ingroup SIMULATION
 */


#ifndef _MOUNTAINCAR_PLANT_H_
#define _MOUNTAINCAR_PLANT_H_

#include "plant.h"

#define MAX_MASSES 5

class MountainCar : public Plant {
 public:
  /** Computes the next state of the plant, given a current state and an action (state transition).  
   * \param state current state of plant.
   * \param action executed action in current state.
   * \param next_state resulting state after executing action.
   * \return true, for success. */ 
  virtual bool get_next_plant_state(const double *current_plant_state, const double *current_action, double *next_plant_state);

  /** Computes a measurement of the plant_state. 
   * \param plant_state     current state
   * \param observed_state  write observation of current state in this array
   * \return true for success */
  virtual bool get_measurement(const double *plant_state, double *measurement);

  /** Notifies that an episode has been started. */
  virtual void notify_episode_starts();

  /** Initialize plant.
   * \param plant_state_dim plant returns dimension of state vector for plant state.
   * \param observation_dim plant returns dimension of observation vector for controller.
   * \param action_dim plant returns dimension of action space. 
   * \param delta_t plant returns duration of one control cycle in seconds.
   * \param fname File, which contains configuration of plant module
   * \return true, for success. */
  virtual bool init(int& _plant_state_dim, int& _action_dim, double& _delta_t, const char *_fname=0, const char *_chapter=0);
  
  /** Terminate plant.
   * \return true for success. */
  virtual void deinit();
    
  /** virtual destructor is necessary since methods declared virtual */
  virtual ~MountainCar();

 protected:
   /** First dimension of the state space: Position of car. */
  static const int S = 0;
  /** Second dimension of the state space: Velocity of car. */
  static const int SP= 1;
  /** Gravity constant */
  static const float G;
  /** Mass of car */
  static const float M; 
  /** Maximum force, that can be applied to the car. */
  double u_max;
  
  double masses[MAX_MASSES];
  int current_mass;

  /** quantization of measurement */
  double quantize;

  /** Minimum position of car. */
//  double pos_min;
  /** Maximum positon of car. */
//  double pos_max;

  /** Duration of one control cycle in seconds. */
  double delta_t;
  /** Number of integration steps used by runge-kutta */
  int no_of_integration_steps;  
 
  /** Reads options for mountain car. */
  bool read_options(const char * fname, const char * chapter);

  /** Computes surface-function H.
   * \param s Input-value to function H
   * \return value of H at s. */ 
  double H(double s);

  /** Computes derivative of H:H'.
   * \param s Input-value to function H'
   * \return value of H' at s. */
  double dHds(double s);

  /** Computes derivative of H':H''. 
   *  \param s Input-value to function H''
   *  \return value of H'' at s. */
  double d2Hds2(double s); 
 
  /** Runge-kutta approximation.
   * Approximates system state after a certain time with respect to the 
   * executed action and the system dynamics.
   * \param start_x Current system state.
   * \param start_u Executed action in state start_x.
   * \param dim Dimension of state space.
   * \param dt Duration of execution of action start_u.
   * \param end_x Approximated system state after time dt. */ 
  void rk4(double *start_x,const double *start_u,int dim,double dt,double *end_x);

  /** Implements system dynamics.  
   * \param x Current system state.
   * \param u Action to be executed.
   * \param dx Derivative of system state with respect to t (time).
   * 
   * The surface of the Mountain-car track is given by a function H:
   * \f[ H(x) := \left\{ \begin{array}{r@{\quad:\quad}l} x^2 + x & x<0 \\
   * \frac{x}{\sqrt{1 + 5x^2}} & x \ge 0 \end{array} \right. \f] 
   *  If \f$ x \f$ denotes the position of the car and \f$ \alpha \f$ the angle between
   *  the surface and a horizontal line , the system dynamics are
   * given by the following differential equation:
   * \f[ \ddot{x}  =  -gsin(\alpha)cos(\alpha) + \frac{F}{m} cos(\alpha) -
   *  \frac{\dot{x}^2}{cos(\alpha)^2}\frac{H'H''}{(1+(H')^2)^2} \f] 
   * The mass of the car is \f$ m \f$ and the applied force is \f$ F \f$. \f$ g \f$ is the
   * gravity constant. */
  void f(double *x,const double* u,double* dx);

};

#endif
