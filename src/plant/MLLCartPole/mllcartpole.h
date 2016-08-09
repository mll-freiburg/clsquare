/*
clsquare - closed loop simulation system
Copyright (c) 2004, Neuroinformatics Group, Prof. Dr. Martin Riedmiller,
University of Osnabrueck

All rights reserved.

Author: Roland Hafner

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

#ifndef _MLLCARTPOLE_H_
#define _MLLCARTPOLE_H_

#include "sysmodel.h"
#include "plant.h"
#include <iostream>

struct MLLCartPole_SysModelParams {
  MLLCartPole_SysModelParams();
  double mc; 	 // mass of the cart                   [kg]
  double mp; 	 // mass of the pendulum               [kg]
  double lp; 	 // length to pendulum center of mass  [m]

  double bp;     // viscous friction factor for pendulum

  double umax;   // maximal motor voltage [V]
  double k2;     // motor torque constant
  double k3;     // motor torque constant
  double Ra;     // motor amature resistant [ohm]
  double Jm;     // motor/gear inertia
  double r;      // radius of pulley wheel
  double n;      // gear factor
  double delay;  // a simulation of delay [0,1) fraction of delta_t
  double k3_0;

  void print(std::ostream& out) {
    out << "--- Model Params: \n"
	<< "mp: " << mp << "\n"
	<< "lp: " << lp << "\n"
	<< "delay: " << delay << "\n"
	<< "k3_0: " << k3_0 << "\n";
  }
};

class MLLCartPole_SysModel: public tools::SysModel
{
 public:
  MLLCartPole_SysModel(double _delta_t, int _num_integration_steps,
			  const MLLCartPole_SysModelParams& _modelparams);
  virtual ~MLLCartPole_SysModel(){;};
  
 protected:
  MLLCartPole_SysModelParams modelparams;
  void sysmodel_derivs(double t, const double *x, double *dxdt, const double *u);
  
};


struct MLLCartPoleParams {
  MLLCartPoleParams();
  
  double  delta_t;
  int     num_integration_steps;
};

/** A cart pole simulation fitted to a real system in the MLL lab.
  *
  * The four-dimensional system state consists of:
  * \li the angle of the pole
  * \li the angular velocity of the pole
  * \li the position of the cart on the track
  * \li the velocity of the cart
  *
  * The one-dimensional action is the force applied to the cart.
  *
  * The general behavior of the plant can be adjusted with:
  * \li \b action_scale_factor (default ): factor that actions are scaled with
  *     before being passed to the system model
  * \li \b observation_mode (default 0): adjusts the content of the plant
  *     measurement; set to 1 to rotate pole angle measurement by 180 degrees;
  *     set to 2 to replace the angle by its sine and cosine (increases measurement
  *     dimensionality by 1)
  * \li \b stop_if_turnover (default 0): if set to anything other than 0,
  *     will abort the episode if a turnover is detected; if set to 1, a
  *     single complete revolution of the pole will count as a turnover,
  *     otherwise two revolutions
  *
  * If all parameters are left at their default values, policies designed
  * for this plant should be applicable to the real system.
  * Nevertheless, its behavior can be adjusted through the following
  * settings:
  * \li \b num_integration_steps (default 100): steps for calculation of next state
  * \li \b delta_t (default 0.02): cycle duration in seconds
  * \li \b delay (default 0.0): a simulation of delay [0,1) fraction of delta_t
  * \li \b mc (default 1.0): mass of the cart (kg)
  * \li \b mp (default 0.1): mass of the pendulum (kg)
  * \li \b lp (default 0.225): length to pendulum center of mass (m)
  * \li \b bp (default 0.0): viscous friction factor for pendulum
  * \li \b umax (default 7.2): maximal motor voltage (V)
  * \li \b k2 (default 0.0389): motor torque constant
  * \li \b k3 (default 0.0389): motor torque constant
  * \li \b k3_0 (default 0.006): motor torque constant
  * \li \b Ra (default 1.23): motor amature resistant (ohm)
  * \li \b Jm (default 0.0001667): motor/gear inertia
  * \li \b r (default 0.02): radius of pulley wheel
  * \li \b n (default 4.8): gear factor
  *
  * @ingroup PLANT
  * @ingroup SIMULATION
  **/

class MLLCartPole : public Plant {
 public:
  virtual bool get_next_plant_state(const double *plant_state, const double *action, double *next_plant_state);
  virtual bool get_measurement(const double *plant_state, double *measurement);
  virtual bool check_initial_state(double *initial_plant_state);
  virtual void notify_episode_starts();
  virtual bool init(int& plant_state_dim, int& measurement_dim, int& action_dim, double& delta_t, const char *fname=0, const char *chapter=0);
  virtual void deinit();
  virtual const std::string get_default_graphics() {return "MLLCartPoleGraphic";};
  
 protected:
  int stop_if_turnover;
  int turnover_state;
  int observation_mode;
  double action_scale_factor;
  MLLCartPole_SysModel       *sysmodel;
  MLLCartPole_SysModelParams modelparams;
  MLLCartPoleParams    plantparams;
  
  double lastu;

  bool read_options(const char* fname, const char* chapter);
};

#endif
