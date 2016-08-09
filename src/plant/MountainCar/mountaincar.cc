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
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  */

#include <math.h>
#include <stdio.h>
#include <iostream>
#include "valueparser.h"
#include "mountaincar.h"

#define X_DIM 2

/** Action default dimension */
#define U_DIM 1

/** The time (for example in seconds) that corresponds to one time tick */
#define DELTA_T_DEFAULT 0.05

/** Gravity constant */
const float MountainCar::G= 9.81;
/** Mass of car */
const float MountainCar::M= 1.0; 

bool MountainCar::get_next_plant_state(const double *current_plant_state, const double *current_action, double *next_plant_state)
{
  // Plant dynamics x(t+1) = f(x,u) 
  double *x;
  
  x = next_plant_state;
  for (int i=0; i < X_DIM; i++){
    x[i]= current_plant_state[i];
  }
  
  double tmp[X_DIM];

  static double *x_tmp[2] = {x,tmp};
  double delta_t_tmp;
  int t;

  int x_idx=0;
  delta_t_tmp = delta_t/(double)no_of_integration_steps;
  x_tmp[x_idx] = x;

  for(t=0;t<no_of_integration_steps;t++){    
    rk4(x_tmp[x_idx],current_action,X_DIM,delta_t_tmp,x_tmp[(x_idx+1)%2]);    
    x_idx = (x_idx+1)%2;
  }
  
  x[S] = x_tmp[x_idx][S];
  x[SP] = x_tmp[x_idx][SP];

  return true;
}

bool MountainCar::get_measurement(const double *plant_state, double *measurement){
  for (int i=0; i<__plant_state_dim; i++) {
    measurement[i]=plant_state[i];
  }
  if(quantize>0){
    measurement[0] = plant_state[0] - fmod(plant_state[0], quantize);
    if(plant_state[0]<0) // always go to lower value
      measurement[0] -= quantize;
    //    cout<<"Quantize. was "<<plant_state[0]<<" now: "<<measurement[0]<<endl;
  }
  return true;
}


void MountainCar::notify_episode_starts(){
    current_mass ++;
    if(current_mass >= MAX_MASSES)
      current_mass = 0;
    if(masses[current_mass] <= 0)
      current_mass = 0;      

    //    IOUT( "current mass: "<<masses[current_mass]<<endl);
}



bool MountainCar::init(int& _plant_state_dim, int& _action_dim, double& _delta_t, const char *_fname, const char *_chapter)
{
  delta_t                  =  DELTA_T_DEFAULT;
  no_of_integration_steps  =  10;
  u_max                    =  5;
//  pos_min                  = -1.2;
//  pos_max                  =  1.2;
  current_mass = -1;
  masses[0] = M;
  
  quantize = 0;

  if(read_options(_fname, _chapter) == false)
    return false;

  // return values:
  _plant_state_dim   = X_DIM;
  _action_dim        = U_DIM;
  _delta_t           = delta_t;
  return true;
}

void MountainCar::deinit()
{
  ;
}

MountainCar::~MountainCar()
{
  ;
}




double MountainCar::H(double s) {
  if (s < 0.0) 
    return  s*s+s;
  return s / sqrt( 1.0 + 5.0 * s * s );
}

double MountainCar::dHds(double s) {
  if(s<0.0)
    return 2.0*s+1.0;
  return 1.0/(pow(1.0 + 5.0*s*s,1.5));
}

double MountainCar::d2Hds2(double s)
{
  if(s<0.0)
    return 2.0;
  return -15.0*s/(pow(1.0 + 5.0*s*s,2.5));
}


void MountainCar::rk4(double *start_x,const double *start_u,int dim,double dt,double *end_x)
{
  int i;
  static double *dx_0=new double[dim];
  static double *dx_1=new double[dim];
  static double *dx_2=new double[dim];
  static double *dx_3=new double[dim];
  static double *x_1=new double[dim];
  static double *x_2=new double[dim];
  static double *x_3=new double[dim];

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
}

void MountainCar::f(double *x,const double *u,double *dx)
{
  static double dH,d2H,epdH2,sin_alpha,cos_alpha,v;
  // static X_t inv_r;
  
  dH = dHds(x[S]);
  d2H = d2Hds2(x[S]);
  epdH2 = 1+dH*dH;
  
  sin_alpha = dH/sqrt(epdH2);
  cos_alpha = 1/sqrt(epdH2);
  
  // Computation of 1/r:
  //   1. r is always measured positively, therefore operations with abs.
  //   2. r can be negative.
  //
  // inv_r=Tools::abs(d2H/pow(epdH2,1.5));
  // inv_r=d2H/pow(epdH2,1.5);
  
  v = x[SP]/cos_alpha;


  // Update internal state
  dx[S] = x[SP];

  // Tangential force u[0] 
  double force = u[0];
  if(force > u_max)
    force = u_max;
  if(force < - u_max)
    force = - u_max;

  double mass = masses[current_mass];

  dx[SP] = -v*v*dH*d2H/(epdH2*epdH2) + force/mass*cos_alpha - G*sin_alpha*cos_alpha; 
}


bool MountainCar::read_options(const char *fname, const char *chapter)
{
  // Optionen Einlesen
  ValueParser vp(fname,chapter==0?"Plant":chapter);

  vp.get("delta_t",delta_t);  
  vp.get("quantize",quantize);  
 // vp.get("pos_min",pos_min);  
 // vp.get("pos_max",pos_max);  
  vp.get("u_max",u_max);  
  vp.get("masses",masses, MAX_MASSES);  
 

  // Show error-message if necessary
  if ( vp.num_of_not_accessed_entries() ) {
    cerr << "\nPlant: not recognized options:";
    vp.show_not_accessed_entries(cerr);
    cerr << endl;
    return false;
  }

  return true;
}



REGISTER_PLANT(MountainCar, "Simulation of a mountain car.")
