/*
clsquare - closed loop simulation system
Copyright (c) 2004, Neuroinformatics Group, Prof. Dr. Martin Riedmiller,
University of Osnabrueck

cartpole.c is based on a previous implementation by Ralf Schoknecht and Artur Merke

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
#include <math.h>


/*****************************************************************
 * PlantPendulum specific parameters
 *****************************************************************/
#define PI     3.14159265
#define G      9.81
#define THETA  0   
#define THETAP 1   
#define S      2  
#define SP     3   
#define X_DIM  4

/** Modified Signum-Function macro */
#define SGN(x)((x)>=0.0? 1.0:-1.0)

/** The number of ingegration steps */
static int no_of_integration_steps;

static float mp=0.1;
static float mc=1.0;
static float mg=1.1;
static float lengthp=.5;
static float frc=.0;
static float frp=.0;
static float delta_t=.02;



/*****************************************************************
 * CartPole Dynamics
 *****************************************************************/

static void f(double* x,const double* u,double* dx)
{
  float sinus ,kosinus,h1, thetapp, spp, fric_pole, fric_cart;

  mg=mc+mp;
  sinus = (float)sin((double)x[THETA]); 
  kosinus = (float)cos((double)x[THETA]);

  // Semantics: h1 == f(t) + mp * lengthp * (thetapunkt)**2 sin(theta) 
  h1 = u[0] + mp * lengthp * x[THETAP] * x[THETAP] * sinus;
  
  fric_pole = frp * x[THETAP]/(mp * lengthp);
  thetapp = ((mg * G * sinus - kosinus * h1 + kosinus*frc*SGN(x[SP])) - fric_pole)/
    (4./3.*mg*lengthp - mp *lengthp *kosinus*kosinus);

  fric_cart = frc* SGN(x[SP]);    
  
  if (fabs(x[SP])<0.001) fric_cart = 0.0;
  
  spp = ((h1 - mp * lengthp *thetapp * kosinus) - fric_cart) / mg ;
  
  // Update internal state
  dx[THETA] = x[THETAP];
  dx[THETAP] = thetapp;
  dx[S] = x[SP];
  dx[SP] = spp;
}


static void rk4(double* start_x,const double* start_u,int dim,float dt,double* end_x)
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

void cartpole_next_state(const double* state, const double* action, 
			 double* next_state){
// Plant dynamics: x(t+1) = f(x,u) 

  double x[X_DIM];
  static double* x_tmp[2] = {x,new double[X_DIM]};
  float dt;
  int t;

  int x_idx=0;
  dt = delta_t /(float)no_of_integration_steps;
  for (int i=0; i< X_DIM; i++) 
    x[i]= state[i];
  x_tmp[x_idx] = x;
  
  for(t=0;t<no_of_integration_steps;t++){
    rk4(x_tmp[x_idx],action,X_DIM,dt,x_tmp[(x_idx+1)%2]);
    x_idx = (x_idx+1)%2;
  }
  
  next_state[THETA] = x_tmp[x_idx][THETA];
  next_state[THETAP] = x_tmp[x_idx][THETAP];
  next_state[S] = x_tmp[x_idx][S];
  next_state[SP] = x_tmp[x_idx][SP];

  // Transfer to values between -PI and PI
  if(next_state[THETA] < - PI)
    next_state[THETA] +=  2.0 * PI;
  else  if(next_state[THETA] > PI)
    next_state[THETA] -= 2.0 * PI;
}

void cartpole_init(const int integration_steps, const float deltat, 
		   const float massc,const float massp, const float lp, 
		   const float fricc, const float fricp){
  no_of_integration_steps = integration_steps;
  delta_t = deltat;
  mc= massc;
  mp= massp;
  lengthp= lp;
  frc=fricc;
  frp=fricp;
}
