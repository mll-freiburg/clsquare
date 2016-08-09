/*
clsquare - closed loop simulation system
Copyright (c) 2004, Neuroinformatics Group, Prof. Dr. Martin Riedmiller,
University of Osnabrueck
Copyright (c) 2011, Machine Learning Lab, Prof. Dr. Martin Riedmiller,
University of Freiburg

demoplant.cc: simple demo plant for demo purposes

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
#include <string.h>
#include "valueparser.h"
#include "demoplant.h"

#define X_DIM 2
#define Y_DIM 2  // dim. of measurement vector
#define U_DIM 1
   
/** The time (for example in seconds) that corresponds to one time tick */
#define DELTA_T_DEFAULT 0.1


/** 
    This file contains the implementation of a simple demo plant that simply adds u to x
**/
   ;


bool DemoPlant::get_next_plant_state(const double *current_plant_state, 
				     const double *current_action, double *next_plant_state){
  
  next_plant_state[0] = current_plant_state[0]+ current_action[0];
  next_plant_state[1] = current_plant_state[1];
  
  if(verbosity){
    IOUT( "transition: ("<<current_plant_state[0]<<", "<<current_plant_state[1]<<")"
	  <<", action: "<<current_action[0]<<" -> ("
	  <<next_plant_state[0]<<", "<<next_plant_state[1]<<")");
  }
  return true;
}


bool DemoPlant::get_measurement(const double *plant_state, double *measurement){
  measurement[0]=plant_state[0] *0.5;
  measurement[1]=42;
  if(verbosity){
    IOUT( "("<<measurement[0]<<", "<<measurement[1]<<")");
  }
  return true;
}

/** default check_initial_state behavior **/
bool DemoPlant::check_initial_state(double *initial_plant_state) 
{
  if(verbosity){
    IOUT( "init state: ("<<initial_plant_state[0]<<", "<<initial_plant_state[1]<<")");
  }
  return true;
}



void DemoPlant::notify_episode_starts(){
  if(verbosity){
    IOUT( "started ");
  }
}


void DemoPlant::notify_episode_stops(){
  if(verbosity){
  IOUT( "stopped ");
  }
}

bool DemoPlant::init(int &plant_state_dim, int &measurement_dim, int &action_dim, 
		 double &delta_t, const char *fname, const char *chapter) {

  __plant_state_dim = X_DIM;
  __measurement_dim = Y_DIM;
  __action_dim = U_DIM;
  __delta_t           = DELTA_T_DEFAULT;

  verbosity = 1;

  if(read_options(fname,chapter) == false)
    return false;

  // return values:
  plant_state_dim   = __plant_state_dim;
  measurement_dim   = __measurement_dim;
  action_dim        = __action_dim;
  delta_t           = __delta_t;
  if(verbosity){
    IOUT( "Successfully initialized ");
  }
  return true;
}


void DemoPlant::notify_command_string(const char* buf){
  IOUT("received command string: "<<buf);
  if(strncmp(buf,"plant_cmd say_hello",18) == 0){
    IOUT("Hello World");
  }
  // in non-simulated plants this one may be important!
  else if(strncmp(buf,"plant_cmd pause",15) == 0){
    IOUT("Sleeping...");
  }
  else{
    EOUT("sorry don't understand command string:"<<buf);
  }
}



bool DemoPlant::read_options(const char *fname, const char *chapter)
{
  // Optionen Einlesen
  ValueParser vp(fname,chapter==0?"Plant":chapter);

  //  vp.get("delta_t",delta_t);  

  vp.get("verbosity",verbosity);  

  // Show error-message if necessary
  if ( vp.num_of_not_accessed_entries() ) {
    cerr << "\nPlant: not recognized options:";
    vp.show_not_accessed_entries(cerr);
    cerr << endl;
    return false;
  }

  return true;
}



REGISTER_PLANT(DemoPlant, "Simulation of a simple Demo Plant.")
