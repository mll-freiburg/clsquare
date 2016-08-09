/*
clsquare - closed loop simulation system
Copyright (c) 2010, 2011 Machine Learning Lab, 
Prof. Dr. Martin Riedmiller, University of Freiburg

Author: Sascha Lange

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
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN 
IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
*/

#include "valueparser.h"
#include "AbstractBatchController.h"
#include "str2val.h"
#include <cstdlib>
#include <fstream>
#include "global.h"
#include <sstream>
#include "random.h"
#include <list>
#include <queue>
#include "setdef.h"
#include <cassert>

using namespace BatchUtils;
using namespace std;

#define MAX_NUM_ACTIONS 255

#define VERBOSE(xxx) if(verbose) cout <<__PRETTY_FUNCTION__ <<": "<< xxx<<endl

#ifdef XCode
#pragma mark -
#pragma mark Object Life Cycle
#endif 


AbstractBatchController::AbstractBatchController() : learn(true)
{}

AbstractBatchController::~AbstractBatchController()
{}




#ifdef XCode
#pragma mark -
#pragma mark Interface to CLSquare
#endif


//void AbstractBatchController::notify_transition(const double* observed_state, const double* action,
//                                                const double* next_observed_state, const double reward,
//                                                const bool is_terminal_state, const double terminal_reward)
//{
//  if (!learn) {
//    return ; // don't need to remember transitions
//  }
//  // store the transition for (later) learning
//  xux.push_back(XUX(observed_state, action, next_observed_state,
//                    is_terminal_state ? reward + terminal_reward : reward, is_terminal_state, observation_dim, action_dim));
//}
void AbstractBatchController::notify_transition(const double* observed_state, const double* action,
                                                const double* next_observed_state, const double reward, 
                                                const bool is_terminal_state, const double terminal_reward)
{
  if (!learn) {
    return ; // don't need to remember transitions
  }
  // store the transition for (later) learning
  xux.push_back(XUX(observed_state, action, next_observed_state, 
                    is_terminal_state ? reward + terminal_reward : reward, is_terminal_state, observation_dim, action_dim));
}

void AbstractBatchController::notify_episode_stops(const double*)
{ 
  if (learn) {
    VERBOSE("Starting the learning procedure at end of episode");
    do_learning(); // call the learning procedure

    if (episodeCount % 10 == 0) {   // write all transition - samples to disk?
      char filename[999];
      sprintf(filename, "samples_%.4d.xux", episodeCount);
      writeXUX(filename);
    }
  }
  
  episodeCount++;
}

void AbstractBatchController::deinit(){  
}

bool AbstractBatchController::init(const int observation_dim, const int action_dim, double deltat, const char* fname, const char* chapter)
{
  
  this->observation_dim = observation_dim; // - numInternalCoords;
  this->action_dim = action_dim;
  verbose = false;
  learn = true;

  episodeCount = 0;
  
  bool result = read_options(fname,chapter);
  srand48(234);    /// \todo add a way to specify specific random seed in config file
  VERBOSE("init batch controller: got observed_observation_dim: "<<this->observation_dim<<" and action_dim:"<<action_dim<<" and delta_t: "<<deltat);
    
  return result;
}


#ifdef XCode
#pragma mark -
#pragma mark Reading and Parsing Options
#endif 



int AbstractBatchController::av2ai(const int* action_value) // helper function for parsing actions
{
  int ai = 0;
  int pot = 1;
  for (int i=0; i < action_dim; i++) {
    ai += action_value[i] * pot;
    pot *= actions_in_dim[i];
  }
  return ai;
}

void AbstractBatchController::av2a(const int* action_value, double* action) // helper function for parsing actions
{
  for (int i =0;i < action_dim;i++) {
    action[i] = action_def[i*MAX_NUM_ACTIONS+action_value[i]];
  }
}

void AbstractBatchController::ai2av(int ai, int* actionValue) 
{
  int pot = 1;
  int tmp = ai;
  for (int i =0;i < action_dim;i++) {
    pot *= actions_in_dim[i];
  }
  for (int i = action_dim-1; i >= 0;i--) {
    pot /= actions_in_dim[i];
    actionValue[i] = tmp / pot;
    tmp = tmp % pot;
  } 
}


static inline void copyArrayToVec(const double* src, vector<double>& trgt, int dim) // copies a c-array to an stl-vector using "the clean way"
{
  assert ((unsigned int)dim == trgt.size());
  for (int i=0; i < dim; i++) {
    trgt[i] = *src++;
  }
}


bool AbstractBatchController::read_options(const char * fname, const char * chapter) {
  char param[255];
  
  ValueParser vp(fname,chapter==NULL?"Controller":chapter);
  vp.get("verbose",verbose);
  vp.get("actions",param,255);
  parse_actions(param);
  vector<double> action(action_dim);
  int actionI[action_dim];
  double actionD[action_dim];
  for (unsigned int i=0; i < num_actions; i++) {
    ai2av(i, actionI);
    av2a(actionI, actionD);
    copyArrayToVec(actionD, action, action_dim);
    all_actions.push_back(action);
  }

  vp.get("learn",learn);
    
  return true;
}

void AbstractBatchController::parse_actions(const char* param)
{    
  const char* str = param;
  double val; 
  action_def = new double[MAX_NUM_ACTIONS*action_dim];  
  actions_in_dim = new int[action_dim];
  num_actions = 0;
  vector<double> action(action_dim);
  
  for (int i = 0; i<action_dim;i++){  
    strskip(str,"[",str);     
    int j = 0;
    while(str2val(str,val,str)) {     
      action_def[i*MAX_NUM_ACTIONS+j] = val;     
      j++;    
    }       
    num_actions += actions_in_dim[i] = j;   
    strskip(str,"]",str);
  }	  	      
}


#ifdef XCode
#pragma mark -
#pragma mark Input / Output
#endif 


void AbstractBatchController::writeXUX(const string& filename) // write all transitions to the file

{
  ofstream out(filename.c_str());
  if (!out) {
    return;
  }
  for (unsigned int i=0; i < xux.size(); i++) {
    out << xux[i] << endl;   
  }
  out.close();
} 


void AbstractBatchController::readXUX(const string& filename) // write all transitions to the file

{
  xux.clear();

  ifstream in(filename.c_str());
  if (!in) {
    return;
  }
  XUX xuxd(observation_dim, action_dim);
  while (in) {
    in >> xuxd;
    if (in) {
    	xux.push_back(xuxd);
    }
  }
  in.close();
}




