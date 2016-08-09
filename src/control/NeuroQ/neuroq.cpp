/*
clsquare - closed loop simulation system
Copyright (c) 2004, Neuroinformatics Group, Prof. Dr. Martin Riedmiller,
University of Osnabrueck

NeuroQ controller
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
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE. 
*/

#include <iomanip>
#include "valueparser.h"
#include "str2val.h"
#include "neuroq.h"

/**********************************************************************/
/* procedures depending on function approximator type                 */
/**********************************************************************/


double NeuroQ::get_Q(const double* state, const int action_idx)
{
  for(int i=0;i<action_net[action_idx].topo_data.in_count;i++){
    action_net[action_idx].in_vec[i] = state[i];
  }
  action_net[action_idx].forward_pass(action_net[action_idx].in_vec,action_net[action_idx].out_vec);
#if 0
  for(int i=0;i<action_net[action_idx].topo_data.in_count;i++)
    cout<<action_net[action_idx].in_vec[i]<<" ";
  cout<<"->"<<action_net[action_idx].out_vec[0]<<endl;
#endif
  return action_net[action_idx].out_vec[0];
}


double NeuroQ::get_Q(const double* state, const double* action)
{  
  for(int i=0;i<qnet.topo_data.in_count;i++){
    if(i<control_state_dim)
      qnet.in_vec[i] = state[i];
    else{
      qnet.in_vec[i] = action[i-control_state_dim];
    }
  }
  qnet.forward_pass(qnet.in_vec,qnet.out_vec);
#if 0
  for(int i=0;i<qnet.topo_data.in_count;i++)
    cout<<qnet.in_vec[i]<<" ";
  cout<<"->"<<qnet.out_vec[0]<<endl;
#endif
  return qnet.out_vec[0];
}

/**********************************************************************/
/* general Q learning procedures                                      */
/**********************************************************************/


bool NeuroQ::in_avoid(const double* state) {
  bool result;
  result = !working_area.isWithinSet(state,state_dim) || avoid_area.isWithinSet(state,state_dim);
  return (result);
}


bool NeuroQ::in_goal(const double* state) {
  return goal_area.isWithinSet(state,state_dim);
}


double NeuroQ::get_greedy_action(const double* state, double* action){
  int best_idx = 0;
  double Qmin=10000; 
  double Qsecond_min=10000; 
  double Q;

  //for(int i=0;i<aset.num_actions;i++){
  
  for(int i=0;i<aset.size();i++){
    double a[action_dim];
    aset.getAction(i , a, action_dim);
    
    if(params.seperate_actionnets==true)
      Q=get_Q(state,i);
    else    
      Q=get_Q(state,a);
    
    double Qtmp = Q;
    if(Q<Qmin){
      best_idx = i;
      Qtmp = Qmin;
      Qmin = Q;
    }
    if(Qtmp<Qsecond_min)
      Qsecond_min = Qtmp;
#if 0
    cout<<i<<":  act "<<aset.action[i][0];
    cout<<" Q : "<<Q<<" best "<<best_idx<<" Qmin "<<Qmin<<endl;
#endif
  }

  double rnum = drand48(); //intention: draw only one random number
  if(rnum < params.exploration){ // do random assignment
    // rnum is a random number between 0 and params.exploration -> scale between 0 and 1
    rnum = rnum/ params.exploration;
    best_idx =(int)(rnum * aset.size());
    if(best_idx==aset.size()) // this might happen if rnum == 1
      best_idx = aset.size()-1;   
  }
  
  for(int i=0;i<action_dim;i++)
    action[i] = aset.action[best_idx][i];
  return Qmin;
}

/**********************************************************************/
/* interface to simusys                                               */
/**********************************************************************/

bool NeuroQ::get_action(const double* observation, double* action){

  double control_state[control_state_dim];

  for(int i= 0; i<control_state_dim;i++)
    control_state[i] = observation[reduced_statevec[i]];

  if(multi_step_ctr == 0){
    get_greedy_action(control_state,action);
    for(int i=0;i<action_dim;i++)
      multi_step_action[i] = action[i];
    multi_step_ctr = multi_step_param ;
  }
  else{ // use same action as before 
  }

  for(int i=0;i<action_dim;i++)
    action[i] = multi_step_action[i];
  multi_step_ctr --;

  if(in_avoid(control_state)){
    //cout<<"**** CONTROLLER: State in Xminus -> stop"<<endl;
    return false;
  }
  if(in_goal(control_state) && stop_in_xplus){
    //cout<<"**** CONTROLLER: State in XPlus -> stop"<<endl;
    return false;
  }

  return true;
}

void NeuroQ::notify_episode_stops(const double* final_observation)
{
  multi_step_ctr = 0;
}


void NeuroQ::deinit(){
  delete[] reduced_statevec;
  delete[] multi_step_action;
}


bool NeuroQ::init(const int observation_dim, const int action_dim, double deltat, const char* fname, const char* chapter)
{
  //defaults
  do_random = false;
  state_dim = observation_dim;
  this->action_dim = action_dim;
  delta_t = deltat;

  multi_step_ctr = 0;
  multi_step_param = 1;  // default: change action every new cycle


  reduced_statevec = new int[state_dim];
  for(int i=0;i<state_dim;i++)
    reduced_statevec[i] = i;
  control_state_dim= state_dim;

  multi_step_action = new double[action_dim];
  for(int i=0;i<action_dim;i++)
    multi_step_action[i] = 0.0;
 
  r = new double[state_dim];
  for(int i=0;i<state_dim;i++)
    r[i]= 0.0;
 
  params.exploration = 0.;
  params.seperate_actionnets = false;
  if(read_options(fname,chapter) == false)
    return false;
  return true;
}

bool NeuroQ::read_options(const char * fname, const char * chapter) {
  if(fname == 0){
    cerr<<"Error: Controller - No .cls file defined"<<endl;
    return false;
  }
  
  ValueParser vp(fname,chapter==NULL?"Controller":chapter);
  char paramstr[MAX_STR_LEN];
  char faname[MAX_STR_LEN];

  if(vp.get("actions",paramstr,MAX_STR_LEN)>=0) {
    //parse_actionstr(paramstr);
    aset.parseStr(paramstr, action_dim);
  }

  bool nfq;
  vp.get("nfqmode", nfq, false);
  
  vp.get("do_random",do_random);
  vp.get("multi_step_param",multi_step_param);
  vp.get("exploration",params.exploration);
  vp.get("seperate_actionnets",params.seperate_actionnets);
  vp.get("separate_actionnets",params.seperate_actionnets,params.seperate_actionnets); // read with previous value as default to correct spelling mistake while preserving compatibility
  vp.get("reload_controlnet",params.reload_controlnet);
  
  vp.get("Qnet",faname,MAX_STR_LEN);
  if(vp.get(nfq?"xplus":"goal_area",paramstr,MAX_STR_LEN)>=0)
    goal_area.parseStr(paramstr,state_dim);
  if(vp.get(nfq?"xminus":"avoid_area",paramstr,MAX_STR_LEN)>=0)
    avoid_area.parseStr(paramstr,state_dim);
  if(vp.get("avoid_area2",paramstr,MAX_STR_LEN)>=0)
    avoid_area.parseStr(paramstr,state_dim);
  if(vp.get(nfq?"xwork":"working_area",paramstr,MAX_STR_LEN)>=0)
    working_area.parseStr(paramstr,state_dim);

  // dummy to avoid warnings
  if(nfq) {
    vp.get("xplusterminal",paramstr,MAX_STR_LEN);
    vp.get("xovershoot",paramstr,MAX_STR_LEN);
  }

  vp.get("stop_in_xplus", stop_in_xplus, true);

  int num_entries =vp.get("reduce_state_to",reduced_statevec,state_dim);
  if(num_entries >0){
    control_state_dim = num_entries;
    for(int i= control_state_dim;i<state_dim;i++)
      reduced_statevec[i] = -1; // invalidate
  }

  if ( vp.num_of_not_accessed_entries() ) {
    stringstream ss;
    ss << "unrecognized options:";
    vp.show_not_accessed_entries(ss);
    ss << endl; 
    WOUT(10, ss.str());
    //return false;
  }

  if(params.seperate_actionnets == true){
    // each action has its own net
    action_net = new Net[aset.size()];
    bool net_loaded = true;
    for(int i=0;i<aset.size();i++){
      char actionnet_name[100];
      sprintf(actionnet_name,"%s.%d",faname,i+1);
      if(action_net[i].load_net(actionnet_name) != OK){
        WOUT(10, "Cannot load action Qnet " << actionnet_name);
        net_loaded = false;
      }
      if(net_loaded==false){
        EOUT("Tried to load action nets, but did not succeed!");
        return false;
      }
    } // for all actions load net
  }
  else{//default: load only one net.
    if(qnet.load_net(faname) != OK)
      return false;
  }

  return true;
}

REGISTER_CONTROLLER(NeuroQ, "A neural network based Q-controller.")
