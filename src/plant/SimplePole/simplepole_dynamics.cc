/*
clsquare - closed loop simulation system
Copyright (c) 2004, Neuroinformatics Group, Prof. Dr. Martin Riedmiller,
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
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE. 
*/

#include <math.h>
#include <iostream> 
#include <stdlib.h>
#include <stdio.h>


/*****************************************************************
 * PlantPendulum specific parameters
 *****************************************************************/
#define PI     3.14159265
#define G      9.81
#define THETA  0   
#define THETAP 1   
#define X_DIM  2
#define RANDOM_RANGE 10.

/** Modified Signum-Function macro */
#define SGN(x)((x)>=0.0? 1.0:-1.0)

/** The number of ingegration steps */
static int no_of_integration_steps = 10;

static double mp;
static double lengthp;
static double delta_t;
static double mu;



/*****************************************************************
 * Simplepole Dynamics
 *****************************************************************/

static void f(double* x,const double* u,double* dx)
{
  /* original equation (communicated by Marc Deisenroth, 30.9.2008)
     dy(1) = y(2);
     dy(2) = (-mu*y(2) + m*g*l*sin(y(1)) + f(t))/(m*l^2);
  */

  double thetapp = (-mu*x[THETAP] + mp * G * lengthp * sin(x[THETA]) + u[0])/ (mp * lengthp * lengthp);
 
  // Update internal state
  dx[THETA] = x[THETAP];
  dx[THETAP] = thetapp;
}


static void rk4(double* start_x,const double* start_u,int dim,double dt,double* end_x)
{
  int i;
  static double* dx_0=new double[dim];
  static double* dx_1=new double[dim];
  static double* dx_2=new double[dim];
  static double* dx_3=new double[dim];
  static double* x_1=new double[dim];
  static double* x_2=new double[dim];
  static double* x_3=new double[dim];

  f(start_x,start_u,dx_0);
  for(i=0;i<dim;i++){
    x_1[i] = start_x[i] + dt/2*dx_0[i];
  }
  f(x_1,start_u,dx_1);
  for(i=0;i<dim;i++){
    x_2[i] = start_x[i] + dt/2*dx_1[i];
  }
  f(x_2,start_u,dx_2);
  for(i=0;i<dim;i++){
    x_3[i] = start_x[i] + dt*dx_2[i];
  }
  f(x_3,start_u,dx_3);

  for(i=0;i<dim;i++){
    end_x[i] = start_x[i] + dt/6 * (dx_0[i] + 2*(dx_1[i]+dx_2[i]) + dx_3[i]);    
  }

  // Testing rk4 only:
  // for(i=0;i<dim;i++)
  //   cout << "\n" << start_x[i] << " " << dx_0[i] << " " << x_1[i] << " " << dx_1[i] << " " << x_2[i]  << " " << dx_2[i] << " " << x_3[i]  << " " << dx_3[i] << " " << end_x[i];
}

void simplepole_next_state_discrete_time(const double* state, const double* action, double* next_state){
/*
dy(:,1) = x(:,1) + dt.*x(:,2) + dt2/2.*(-mu.*x(:,2) + (m*g*l).*sin(x(:,1)) + u)./(m*l2);
dy(:,2) = x(:,2) + dt.*(-mu.*x(:,2) + (m*g*l).*sin(x(:,1)) + u)./(m*l2); 
*/

  double thetapp =  (-mu * state[THETAP] + (mp *G * lengthp) * sin(state[THETA]) + action[0])/ (mp * lengthp * lengthp);
  next_state[THETA] = state[THETA] + delta_t * state[THETAP] + delta_t *delta_t/2.0 * thetapp;
  next_state[THETAP] = state[THETAP] + delta_t * thetapp;

  // Transfer to values between -PI and PI
  if(next_state[THETA] < - PI)
    next_state[THETA] +=  2.0 * PI;
  else  if(next_state[THETA] > PI)
    next_state[THETA] -= 2.0 * PI;
}



void simplepole_next_state(const double* state, const double* action, double* next_state){
// Plant dynamics: x(t+1) = f(x,u) 

  double x[X_DIM];
  static double* x_tmp[2] = {x,new double[X_DIM]};
  double dt;
  int t;
  double tmp_action[1];

  tmp_action[0]= action[0]*2*RANDOM_RANGE-RANDOM_RANGE; // just copy

  int x_idx=0;
  dt = delta_t /(double)no_of_integration_steps;
  for (int i=0; i< X_DIM; i++) 
    x[i]= state[i];
  x_tmp[x_idx] = x;
  
  for(t=0;t<no_of_integration_steps;t++){
    rk4(x_tmp[x_idx],tmp_action,X_DIM,dt,x_tmp[(x_idx+1)%2]);
    x_idx = (x_idx+1)%2;
  }
  
  next_state[THETA] = x_tmp[x_idx][THETA];
  next_state[THETAP] = x_tmp[x_idx][THETAP];

  // Transfer to values between -PI and PI
  if(next_state[THETA] < - PI)
    next_state[THETA] +=  2.0 * PI;
  else  if(next_state[THETA] > PI)
    next_state[THETA] -= 2.0 * PI;
}

void simplepole_init(const double deltat, const double massp, const double lp, const double _mu){
  delta_t = deltat;
  mp= massp;
  lengthp= lp;
  mu = _mu;
}
