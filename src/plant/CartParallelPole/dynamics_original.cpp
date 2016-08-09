/*
clsquare - closed loop simulation system
Copyright (c) 2012, Neuroinformatics Group, Prof. Dr. Martin Riedmiller,
University of Osnabrueck

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
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

#include <cmath>
#include "dynamics_original.h"

#define G      9.81
#define THETA  0
#define THETAP 1
#define S      2
#define SP     3
#define X_DIM  4

/** Modified Signum-Function macro */
#define SGN(x)((x)>=0.0? 1.0:-1.0)

/*****************************************************************
 * CartPole Dynamics
 *****************************************************************/

void ParallelPoleDynamicsOR::f (double* x, const double* u, double* dx)
{
  double sinus, kosinus, h1, thetapp, spp, fric_pole, fric_cart;

  double mg = _MASS_CART + _mp;
  sinus = (double)sin((double)x[THETA]);
  kosinus = (double)cos((double)x[THETA]);

  // Semantics: h1 == f(t) + mp * lengthp * (thetapunkt)**2 sin(theta)
  h1 = u[0] + _mp * _lengthp * x[THETAP] * x[THETAP] * sinus;

  fric_pole = _frp * x[THETAP] / (_mp * _lengthp);
  thetapp = ((mg * G * sinus - kosinus * h1 + kosinus * _FRIC_CART * SGN(x[SP])) - fric_pole) / (4. / 3. * mg * _lengthp - _mp * _lengthp * kosinus * kosinus);

  fric_cart = _FRIC_CART * SGN(x[SP]);

  if (fabs(x[SP])<0.001) fric_cart = 0.0;

  spp = ((h1 - _mp * _lengthp * thetapp * kosinus) - fric_cart) / mg ;

  // Update internal state
  dx[THETA] = x[THETAP];
  dx[THETAP] = thetapp;
  dx[S] = x[SP];
  dx[SP] = spp;
}

void ParallelPoleDynamicsOR::rk4 (double* start_x, const double* start_u, int dim, double dt, double* end_x)
{
  int i;
  static double* dx_0 = new double[dim];
  static double* dx_1 = new double[dim];
  static double* dx_2 = new double[dim];
  static double* dx_3 = new double[dim];
  static double* x_1  = new double[dim];
  static double* x_2  = new double[dim];
  static double* x_3  = new double[dim];

  f(start_x,start_u,dx_0);
  for (i=0; i<dim; i++)
    x_1[i] = start_x[i] + dt/2*dx_0[i];
  f(x_1,start_u,dx_1);
  for (i=0; i<dim; i++)
    x_2[i] = start_x[i] + dt/2*dx_1[i];
  f(x_2,start_u,dx_2);
  for (i=0; i<dim; i++)
    x_3[i] = start_x[i] + dt*dx_2[i];
  f(x_3,start_u,dx_3);
  for (i=0; i<dim; i++)
    end_x[i] = start_x[i] + dt/6 * (dx_0[i] + 2*(dx_1[i]+dx_2[i]) + dx_3[i]);
}

#define SET_POLE_VARS(_num_) \
  _lengthp = _LENGTH_POLE##_num_; \
  _mp = _MASS_POLE##_num_; \
  _frp = _FRIC_POLE##_num_;
void ParallelPoleDynamicsOR::next_state (const double* state, const double* action, double* next_state)
{
  if (_DOUBLE) {
    double tmp_state[4] = {state[0], state[1], state[4], state[5]};
    SET_POLE_VARS(1);
    next_state_intern(tmp_state, action, next_state);

    double tmp_next[2] = {next_state[2], next_state[3]};
    SET_POLE_VARS(2);
    next_state_intern(&state[2], action, &next_state[2]);

    next_state[4] = (next_state[4] + tmp_next[0]) / 2.;
    next_state[5] = (next_state[5] + tmp_next[1]) / 2.;
    ///< \todo this is physically silly! state of cart should be calculated based on cumulative weight, not averaged
  } else {
    SET_POLE_VARS(1);
    next_state_intern(state, action, next_state);
  }
}

void ParallelPoleDynamicsOR::next_state_intern (const double* state, const double* action, double* next_state)
{
  // Plant dynamics: x(t+1) = f(x,u)

  double x[X_DIM];
  static double* x_tmp[2] = {x,new double[X_DIM]};
  double dt;
  int t;
  int x_idx=0;

  dt = _DELTA / (double)_no_of_integration_steps;
  for (int i=0; i< X_DIM; i++)
    x[i]= state[i];
  x_tmp[x_idx] = x;

  for (t=0; t<_no_of_integration_steps; t++) {
    rk4(x_tmp[x_idx],action,X_DIM,dt,x_tmp[(x_idx+1)%2]);
    x_idx = (x_idx+1)%2;
  }

  next_state[THETA]  = x_tmp[x_idx][THETA];
  next_state[THETAP] = x_tmp[x_idx][THETAP];
  next_state[S]  = x_tmp[x_idx][S];
  next_state[SP] = x_tmp[x_idx][SP];

  // Transfer to values between -PI and PI
  if (next_state[THETA] < -M_PI)
    next_state[THETA] += 2.0 * M_PI;
  else  if (next_state[THETA] > M_PI)
    next_state[THETA] -= 2.0 * M_PI;
}

