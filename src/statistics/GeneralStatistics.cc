/*
clsquare - closed loop simulation system
Copyright (c) 2004, Neuroinformatics Group, Prof. Dr. Martin Riedmiller,
University of Osnabrueck

Author: Martin Riedmiller

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
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  */

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include "valueparser.h"
#include "str2val.h"
#include "GeneralStatistics.h"
#include "global.h"


#define HM_ONLYONCE 1
#define HM_EVERYTIME 2
#define HM_EVERYTIME_PLUS_EPISODECTR 3

//#define OUTPUT(XXX) std::setw(15) << std::setprecision(5)<< std::fixed<< XXX
#define OUTPUT(XXX) std::setw(12) << std::setprecision(2)<< std::fixed<< XXX

void GeneralStatistics::init_currPerf(){
  currPerf.num_cycles = 0;
  currPerf.touchedXplus = false;
  currPerf.touchedXminus = false;
  currPerf.touchedOutofXwork = false;
  currPerf.terminatedInXplus = false;
  currPerf.crashed = false; 
  currPerf.cyclesOutofXplus = 0;
  currPerf.cyclesInXplus = 0;
  currPerf.cyclesUntilCrashed = 0;
  currPerf.cyclesUntilReachedXplus = 0;
  currPerf.cyclesBeforeAsymptotic = 0;
  currPerf.costs = 0;
  currPerf.reward = 0;
  currPerf.final_reward = 0;
  currPerf.last_cycles_error_sum  = 0.0;
  currPerf.last_cycles_error_ctr  = 0;
}

void GeneralStatistics::init_accPerf(){
  accPerf.touchedXplus = 0;
  accPerf.touchedXminus = 0;
  accPerf.touchedOutofXwork = 0;
  accPerf.terminatedInXplus = 0;
  accPerf.crashed = 0;  
  accPerf.maxCyclesBeforeAsymptotic = 0;
  accPerf.average_cyclesOutofXplus = 0;
  accPerf.average_cyclesInXplus = 0;
  accPerf.average_cyclesUntilCrashed = 0;
  accPerf.average_cyclesUntilReachedXplus = 0;
  accPerf.average_costs = 0;
  accPerf.average_reward = 0;
  accPerf.average_final_reward = 0;
  accPerf.cyclesTotal = 0;
  accPerf.num_episodes = 0;
  accPerf.avg_last_cycles_err = 0.0;
  accPerf.min_last_cycles_err = 0.0;
  accPerf.max_last_cycles_err = 0.0;
}

void GeneralStatistics::update_currPerf(const double* state, const double* action, const double reward){  
  currPerf.num_cycles++;  
  
  currPerf.reward += reward;
  if (currPerf.touchedXplus == false)
    currPerf.cyclesUntilReachedXplus++; 
  
  // check status of state:
  if(XplusSet.isWithinSet(state, state_dim) && XworkSet.isWithinSet(state, state_dim)){
    currPerf.touchedXplus = true;    
    currPerf.cyclesInXplus++;  
  }
  else{
    currPerf.costs++;
    currPerf.cyclesOutofXplus++;  
    currPerf.cyclesBeforeAsymptotic = currPerf.num_cycles;
    if (XminusSet.isWithinSet(state, state_dim))
      currPerf.touchedXminus = true;
    if (XworkSet.numSubsets() >0 && ! XworkSet.isWithinSet(state, state_dim))
      currPerf.touchedOutofXwork = true;    
  }
  
  /** \todo reward in stat behandeln */
  //currPerf.reward+= reward.get_reward(state, action);

  if(currPerf.touchedXminus || currPerf.touchedOutofXwork) {
    currPerf.crashed = true;
    currPerf.costs = MAX_COSTS;
  }
  
  if(currPerf.crashed == false)
    currPerf.cyclesUntilCrashed ++;   

  if (err_obs_value_id >= 0 && currPerf.num_cycles > err_start_cycle) {
    currPerf.last_cycles_error_sum += fabs(state[err_obs_value_id]);
    currPerf.last_cycles_error_ctr++;
  }
  
}

void GeneralStatistics::finish_episode(const double* state){
  if(XplusSet.isWithinSet(state,state_dim) && XworkSet.isWithinSet(state,state_dim)){
    currPerf.terminatedInXplus = true;
  } 
  if(currPerf.crashed == false)
    currPerf.cyclesUntilCrashed= max_episode_length;  
  else {   
    currPerf.cyclesBeforeAsymptotic = max_episode_length;
    currPerf.cyclesOutofXplus += max_episode_length - currPerf.num_cycles;
  } 
}

void GeneralStatistics::update_accPerf(){
  accPerf.cyclesTotal += currPerf.num_cycles;
  accPerf.num_episodes ++; 
  if(currPerf.touchedXplus) 
     accPerf.touchedXplus++; 
  if(currPerf.touchedXminus) 
    accPerf.touchedXminus++;
  if(currPerf.touchedOutofXwork) 
    accPerf.touchedOutofXwork++;
  if(currPerf.terminatedInXplus) 
    accPerf.terminatedInXplus++;
  if(currPerf.crashed) 
    accPerf.crashed++;
    
  // recurrent computation of average value: xquer_n = (1-alpha) * xquer_n-1 + alpha * x_n
  double alpha = 1.0/ (double)(accPerf.num_episodes);
  accPerf.average_cyclesOutofXplus = (1-alpha) * accPerf.average_cyclesOutofXplus + alpha * currPerf.cyclesOutofXplus;
  accPerf.average_cyclesInXplus = (1-alpha) * accPerf.average_cyclesInXplus + alpha * currPerf.cyclesInXplus;
  accPerf.average_cyclesUntilCrashed = (1-alpha) * accPerf.average_cyclesUntilCrashed 
    + alpha * currPerf.cyclesUntilCrashed;  
  accPerf.average_cyclesUntilReachedXplus = (1-alpha) * accPerf.average_cyclesUntilReachedXplus 
    + alpha * currPerf.cyclesUntilReachedXplus;
  accPerf.maxCyclesBeforeAsymptotic = (1-alpha)*accPerf.maxCyclesBeforeAsymptotic 
    + alpha * currPerf.cyclesBeforeAsymptotic;
  accPerf.average_costs = (1-alpha)*accPerf.average_costs
    + alpha * currPerf.costs;
  accPerf.average_reward = (1-alpha)*accPerf.average_reward
    + alpha * currPerf.reward;
  accPerf.average_final_reward = (1-alpha)*accPerf.average_final_reward
    + alpha * currPerf.final_reward;

  if ( currPerf.last_cycles_error_ctr > 0) {
    double avg_last_cycles_err = currPerf.last_cycles_error_sum / (double) currPerf.last_cycles_error_ctr;

    // std::cerr << " avg_last_cycles_err : " << currPerf.last_cycles_error_sum << " / " << currPerf.last_cycles_error_ctr << " -> " <<avg_last_cycles_err  << "\n";
    
    accPerf.avg_last_cycles_err = (1-alpha) * accPerf.avg_last_cycles_err + alpha * avg_last_cycles_err;
   
    // std::cerr << "Avg: " <<  accPerf.avg_last_cycles_err << "\n";

    if (accPerf.num_episodes==1) 
      {
	accPerf.min_last_cycles_err = avg_last_cycles_err;
	accPerf.max_last_cycles_err = avg_last_cycles_err;
      }
    else 
      {
	if (avg_last_cycles_err > accPerf.max_last_cycles_err) accPerf.max_last_cycles_err = avg_last_cycles_err;
	if (avg_last_cycles_err < accPerf.min_last_cycles_err) accPerf.min_last_cycles_err = avg_last_cycles_err;
      }
    
  }
  
}


void GeneralStatistics::print_meaning(){
  //    *out << "# CLSquare GeneralStatistics file. Standardized Benchmarking format.\n";

  *out   
    << OUTPUT("# Fail")
    <<OUTPUT("X+ Touch")
    <<OUTPUT("X+ Term")
    <<OUTPUT("X- Term")
    <<OUTPUT("length")
    <<OUTPUT("OutOfX+")
    <<OUTPUT("InX+")
    <<OUTPUT("Avg. cost")
    <<OUTPUT("Avg. R")
    <<OUTPUT("Avg. R_fin")
    <<OUTPUT("bef. asym")
    //    <<OUTPUT("total cycles")
    //    <<OUTPUT("num episode")
    <<OUTPUT("avg. err.")
    <<OUTPUT("min err.")
    <<OUTPUT("max err.");
  *out << endl;
 }

bool GeneralStatistics::open_file()
{
  if(GeneralStatistics_mode <0)
    return true;
 
  out = new ofstream(ProtFName);
  if (!out || !*out) {
    EOUT("Can't open file: (" << ProtFName << ")!");
    return false;
  }
       
  
  if(GeneralStatistics_mode == 0 && header_mode == HM_ONLYONCE){
    print_meaning();
  }
  return(true);
}

void GeneralStatistics::write2file_mode0(const long episode){
 double N;
  N= (double) accPerf.num_episodes;
  if(N==0) // already written
    return;  
  
  if(header_mode == HM_EVERYTIME_PLUS_EPISODECTR)
    *out<<endl<<"# episode no: "<<episode<<", averaged over "<<accPerf.num_episodes<<" episodes in "<< accPerf.cyclesTotal <<" cycles total."<<endl;

  if(header_mode == HM_EVERYTIME || header_mode == HM_EVERYTIME_PLUS_EPISODECTR)
    print_meaning(); 


  *out 
    <<OUTPUT(accPerf.crashed / N * 100.)
    <<OUTPUT(accPerf.touchedXplus / N *100.)
    <<OUTPUT(accPerf.terminatedInXplus/ N * 100.)
    <<OUTPUT(accPerf.touchedXminus/ N * 100.)
    <<OUTPUT(accPerf.average_cyclesUntilCrashed)
    <<OUTPUT(accPerf.average_cyclesOutofXplus)
    <<OUTPUT(accPerf.average_cyclesInXplus)
    <<OUTPUT(accPerf.average_costs)
    <<OUTPUT(accPerf.average_reward)
    <<OUTPUT(accPerf.average_final_reward)
    <<OUTPUT(accPerf.maxCyclesBeforeAsymptotic)
    //    <<OUTPUT(accPerf.cyclesTotal)
    //    <<OUTPUT(accPerf.num_episodes) 
    <<OUTPUT(accPerf.avg_last_cycles_err)
    <<OUTPUT(accPerf.min_last_cycles_err)
    <<OUTPUT(accPerf.max_last_cycles_err)
    <<endl;
}

void GeneralStatistics::write2file_mode1(const long episode){
 double N;
  N= (double) accPerf.num_episodes;
  if(N==0) // already written
    return;  
  
  *out << " ";  // this compensates the '#' sign in comment lines
  *out 
    <<OUTPUT(accPerf.terminatedInXplus/ N * 100.)
    <<OUTPUT(accPerf.crashed / N * 100.)
    <<OUTPUT(accPerf.average_cyclesOutofXplus)
    <<OUTPUT(accPerf.average_cyclesUntilCrashed)
    <<OUTPUT(accPerf.touchedXplus/ N * 100.)
    <<OUTPUT(accPerf.touchedXminus/ N * 100)
    <<OUTPUT(accPerf.touchedOutofXwork/ N * 100.)
    <<OUTPUT(accPerf.average_reward)
    <<OUTPUT(accPerf.average_final_reward)
    <<OUTPUT(accPerf.num_episodes)
    <<OUTPUT(accPerf.avg_last_cycles_err)
    <<OUTPUT(accPerf.min_last_cycles_err)
    <<OUTPUT(accPerf.max_last_cycles_err)
    <<endl;	    	       
}

void GeneralStatistics::write2file(const long episode){   
  switch(GeneralStatistics_mode){
  case 0:
    write2file_mode0(episode);
    break;
  case 1:
    write2file_mode1(episode);
    break;  
  default:
    break;
  }
  init_accPerf();
}

/*****************************************************************
 * GeneralStatistics
 *****************************************************************/

void GeneralStatistics::notify(const double* plant_state, const double* measurement, const double* observation, const double* action, const long cycle, const long episode,
                        const float total_time, const float episode_time, const long total_num_of_cycles, double transition_rating_signal){
  if(GeneralStatistics_mode <0)
    return;

  switch (state_mode) {
    case PlantState:
      update_currPerf(plant_state, action, transition_rating_signal);
      break;
    case Observation:
      update_currPerf(observation, action, transition_rating_signal);
      break;
    case Measurement:
    default:
      update_currPerf(measurement, action, transition_rating_signal);
  }
}

void GeneralStatistics::notify_episode_starts(const long episode_ctr){
  init_currPerf();
}

void GeneralStatistics::notify_episode_stops(const double* plant_state, const double* measurement, const double* observation,
                             const long cycle, const long episode,
                             const float total_time, const float episode_time, const long total_num_of_cycles, bool is_terminal, double terminal_rating_signal){
  switch (state_mode) {
    case PlantState:
      finish_episode(plant_state);
      break;
    case Observation:
      finish_episode(observation);
      break;
    case Measurement:
    default:
      finish_episode(measurement);
  }
  
  //  cout<<"terminal rating signal"<<terminal_rating_signal<<endl;
  currPerf.final_reward = terminal_rating_signal;
  update_accPerf();
  if(averageOverNEpisodes >0 && (episode % averageOverNEpisodes) == 0)
    write2file(episode);
}

void GeneralStatistics::deinit(){
  if(averageOverNEpisodes <=0)
    write2file(accPerf.num_episodes);
  if (out!=0) {
    *out<< std::flush;
    delete out;
  }
}

bool GeneralStatistics::init(int _plant_state_dim, int _measurement_dim, int _observation_dim, int _action_dim, double _delta_t, long _cycles_per_episode, const char *fname){
  //values
  max_episode_length  = _cycles_per_episode;
  state_dim           = _measurement_dim;
  action_dim          = _action_dim;
  delta_t             = _delta_t;
  header_mode = HM_ONLYONCE;
  state_mode  = Measurement;

  //defaults
  out                  = 0;
  noheader             = false;
  sprintf(ProtFName,"%s","default.stat"); 
  GeneralStatistics_mode      = -1; // no GeneralStatistics

  //if(reward.init(state_dim, action_dim, delta_t, fname) == false) // init the reward object
  //return false;

  err_obs_value_id = -1;
  err_start_cycle  =  0;

  plant_state_dim = _plant_state_dim;
  observation_dim = _observation_dim;
  measurement_dim = _measurement_dim;
 
  read_options(fname);
  if (noheader == true)
    header_mode = 0;

  if(GeneralStatistics_mode >= 0){
    if(open_file()==false)
      return false;
  }
  init_accPerf();
  init_currPerf();
  return true;
}

bool GeneralStatistics::read_options(const char * fname) 
{
  char paramstr[MAX_STR_LEN];
  bool xplus_asin_control = false;
  bool xwork_asin_control = false;
  bool xminus_asin_control = false;

  if(fname == 0)
    return true;

  //  ValueParser vp(fname,"GeneralStatistics");
  ValueParser vp(fname,"Statistics");
  if (vp.get("input_type",paramstr,MAX_STR_LEN)>=0){
    if(strcmp(paramstr,"plant_state")==0){
      IOUT("Generating statistics over plant state.");
      state_mode = PlantState;
      state_dim = plant_state_dim;
    } else if (strcmp(paramstr,"measurement")==0) {
      IOUT("Generating statistics over measurement.");
      state_mode = Measurement;
      state_dim = measurement_dim;
    } else if (strcmp(paramstr,"observation")==0) {
      IOUT("Generating statistics over observation.");
      state_mode = Observation;
      state_dim = observation_dim;
    }
  }
  if(vp.get("xplus",paramstr,MAX_STR_LEN)>=0){
    if(strcmp(paramstr,"as_in_control")==0)
      xplus_asin_control = true;
    else
      XplusSet.parseStr(paramstr, state_dim);
  }
  if(vp.get("xminus",paramstr,MAX_STR_LEN)>=0){
    if(strcmp(paramstr,"as_in_control")==0)
      xminus_asin_control = true;
    else
      XminusSet.parseStr(paramstr, state_dim);
  }
  if(vp.get("xwork",paramstr,MAX_STR_LEN)>=0){
    if(strcmp(paramstr,"as_in_control")==0)
      xwork_asin_control = true;
    else
      XworkSet.parseStr(paramstr, state_dim);
  }
  vp.get("average_over_n_episodes", averageOverNEpisodes); 
  vp.get("average_over_n_episodes", averageOverNEpisodes); 
  if(vp.get("statistics_mode",paramstr,MAX_STR_LEN)>=0){
    if(strcmp(paramstr,"standardized")== 0)
      GeneralStatistics_mode = 0; 
    if(strcmp(paramstr,"raw")== 0)
      GeneralStatistics_mode = 1;    
    // default: -1 (none)
  }
  vp.get("noheader", noheader); 
  vp.get("header_mode", header_mode); 
  
  vp.get("statistics_file",ProtFName,MAX_STR_LEN);

  vp.get("err_obs_value_id" , err_obs_value_id);

  if (err_obs_value_id > 0) {
    if (err_obs_value_id >= state_dim) {
      EOUT("Param: err_obs_value_id : " << err_obs_value_id << " bigger than obs_dim: " << state_dim);
      return false;
    }
    vp.get("err_start_cycle" , err_start_cycle);
  }

  ValueParser vp1(fname,"Controller");
  if(xplus_asin_control==true){
    if(vp1.get("xplus",paramstr,MAX_STR_LEN)>=0)
      XplusSet.parseStr(paramstr, state_dim);
  }
  if(xwork_asin_control==true){
    if(vp1.get("xwork",paramstr,MAX_STR_LEN)>=0)
      XworkSet.parseStr(paramstr, state_dim);
  }
  if(xminus_asin_control==true){
    if(vp1.get("xminus",paramstr,MAX_STR_LEN)>=0)
      XminusSet.parseStr(paramstr, state_dim);
  }

  return true;
}

REGISTER_STATISTICS(GeneralStatistics , "A general statistics module with mixed modes and options.")
