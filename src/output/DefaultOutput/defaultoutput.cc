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
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE. 
*/

#include <stdio.h>
#include <iostream>
#include <string.h>
#include <sys/time.h>
#include <fstream>
#include <iomanip>

#include "valueparser.h"
#include "defaultoutput.h"
#include "global.h"

void DefaultOutput::print_observation(std::ostream& _out, const double* _observation) 
{
  for (unsigned int i=0; i<selected_observation_ids.size(); i++)
    _out << std::setw(outp_width) << std::setprecision(outp_prec)
         << _observation[selected_observation_ids[i]] << "  ";
}

void DefaultOutput::print_measurement(std::ostream& _out, const double* _measurement)
{
  for (unsigned int i=0; i<selected_measurement_ids.size(); i++)
    _out << std::setw(outp_width) << std::setprecision(outp_prec)
         << _measurement[selected_measurement_ids[i]] << " ";
}

void DefaultOutput::print_action(std::ostream& _out, const double* _action) 
{
  for (unsigned int i=0; i<selected_action_ids.size(); i++)
    _out << std::setw(outp_width) << std::setprecision(outp_prec)
         << _action[selected_action_ids[i]] << "  ";
}

void DefaultOutput::print_plant_state(std::ostream& _out, const double* _plant_state) 
{
  for (unsigned int i=0; i<selected_plant_state_ids.size(); i++)
    _out << std::setw(outp_width) << std::setprecision(outp_prec)
         << _plant_state[selected_plant_state_ids[i]] << "  ";
}

void DefaultOutput::print_external_signal(std::ostream& _out, const double* _external_signal)
{
  for (unsigned int i=0; i<selected_external_signal_ids.size(); i++)
    _out << std::setw(outp_width) << std::setprecision(outp_prec)
         << _external_signal[selected_external_signal_ids[i]] << "  ";
}

void DefaultOutput::print_cycle_info(std::ostream& _out, const sys_time_struct& ti) {
  _out << std::setw(outp_width) << std::setprecision(outp_prec)<< ti.cycle_ctr       << " ";
  _out << std::setw(outp_width) << std::setprecision(outp_prec)<< ti.episode_time    << " ";
  _out << std::setw(outp_width) << std::setprecision(outp_prec)<< ti.total_time      << " ";
  _out << std::setw(outp_width) << std::setprecision(outp_prec)<< ti.total_real_time << " ";
  _out << " : ";
}

void DefaultOutput::print_header(std::ostream& _out)
{
  _out << "#";

  _out << "cycleid ";
  _out << "episode_time ";
  _out << "total_time ";
  _out << "real_total_time ";
  _out << " : ";

  switch( output_mode ) {
  case 0:
    _out << "observation(t-1)[";
    for (unsigned int i=0; i<selected_observation_ids.size(); i++) _out << selected_observation_ids[i] << " ";
    _out << "] ";
    _out << " | ";
    _out << "action(t-1)[";
    for (unsigned int i=0; i<selected_action_ids.size(); i++) _out << selected_action_ids[i] << " ";
    _out << "] ";
    _out << " | ";
    _out << "observation(t)[";
    for (unsigned int i=0; i<selected_observation_ids.size(); i++) _out << selected_observation_ids[i] << " ";
    _out << "] ";
    break;
  case 1:
    _out << "observation[";
    for (unsigned int i=0; i<selected_observation_ids.size(); i++) _out << selected_observation_ids[i] << " ";
    _out << "] ";
    _out << " | ";
    _out << "action[";
    for (unsigned int i=0; i<selected_action_ids.size(); i++) _out << selected_action_ids[i] << " ";
    _out << "] ";
    if (selected_plant_state_ids.size()>0) {
      _out << " | plant_state[ ";
      for (unsigned int i=0; i<selected_plant_state_ids.size(); i++) _out << selected_plant_state_ids[i] << " ";
      _out << "] ";
    }
    if (selected_measurement_ids.size()>0) {
      _out << " | measurement[ ";
      for (unsigned int i=0; i<selected_measurement_ids.size(); i++) _out << selected_measurement_ids[i] << " ";
      _out << "] ";
    }
    if (selected_external_signal_ids.size()>0) {
      _out << " | external_signal[ ";
      for (unsigned int i=0; i<selected_external_signal_ids.size(); i++) _out << selected_external_signal_ids[i] << " ";
      _out << "] ";
    }
    _out << " | r <transition_rating> or R <terminal_rating> or empty ";
    break;
  };
  _out << "\n";
}

void DefaultOutput::print_episode_header(std::ostream& _out, long int _episode)
{
  _out << "\n\n# Started episode No. " << _episode << "\n";
}


// write xux information
void DefaultOutput::write_om0(std::ostream& _out, const double* _prev_observation, const double* _prev_action, const double* _observation)
{ 
  print_observation( _out , _prev_observation );
  _out << " | ";
  print_action( _out , _prev_action );
  _out << " | ";
  print_observation( _out , _observation );
  _out << "\n";
}

// write all information (dependend on parameters)
void DefaultOutput::write_om1(std::ostream&                _out,
                       const sys_state_variables&   _state_vars ,
                       const double*                _action,
                       double                       _transition_rating ) {
  print_observation( _out , _state_vars.observed_state );
  _out << " | ";
  print_action( _out , _action );
  if (selected_plant_state_ids.size()>0) {
    _out << " | ";
    print_plant_state( _out , _state_vars.plant_state );
  }
  if (selected_measurement_ids.size()>0) {
    _out << " | ";
    print_measurement( _out , _state_vars.measurement );
  }
  if (selected_external_signal_ids.size()>0) {
    _out << " | ";
    print_external_signal( _out , _state_vars.external_signal );
  }
  /*
  _out << " | r ";
  _out << _transition_rating ;
  */
  _out << "\n";
}

void DefaultOutput::write_om1_last(std::ostream&                _out,
                            const sys_state_variables&   _state_vars ,
                            bool                         _terminal,
                            double                       _terminal_rating ){

  print_observation( _out , _state_vars.observed_state );

  if (selected_plant_state_ids.size()>0) {
    _out << " | ";
    print_plant_state( _out , _state_vars.plant_state );
  }
  if (selected_measurement_ids.size()>0) {
    _out << " | ";
    print_measurement( _out , _state_vars.measurement );
  }
  if (selected_external_signal_ids.size()>0) {
    _out << " | ";
    print_external_signal( _out , _state_vars.external_signal );
  }
  if (_terminal) {
    _out << " | R ";
    _out << _terminal_rating;
  }
  _out << "\n";
}




/*****************************************************************
 * DefaultOutput
 *****************************************************************/

void DefaultOutput::notify               ( const sys_state_variables&     current_state_vars,
                                    const double*                  current_action,
                                    const sys_time_struct&         current_sys_time,
                                    double                         transition_rating) {
  if(output_mode <0) return;

  long cycle   = current_sys_time.cycle_ctr;
  long episode = current_sys_time.episode_ctr;

  if(cycle == 1 && episode == 1) print_header(*out);
  if(cycle == 1) print_episode_header(*out, episode);

  if(output_mode == 0) {
    if (cycle>1) {
      print_cycle_info(*out, current_sys_time);
      write_om0(*out, prev_observation,  prev_action, current_state_vars.observed_state );
    }
  } else if(output_mode == 1) {
    print_cycle_info(*out, current_sys_time);
    write_om1(*out, current_state_vars, current_action, transition_rating);
  } else {
    CLSERR("Unknown output mode! " << output_mode );
  }

  *out << std::flush;

  // store information
  for(int i=0;i<observation_dim;i++)
    prev_observation[i]= current_state_vars.observed_state[i];
  for(int i=0;i<action_dim;i++)
    prev_action[i]=current_action[i];
}

void DefaultOutput::notify_episode_stops ( const sys_state_variables&     current_state_vars,
                                    const sys_time_struct&         current_sys_time,
                                    bool                           terminal,
                                    double                         terminal_rating)
{
  if(output_mode <0) return;

  long cycle   = current_sys_time.cycle_ctr;
  long episode = current_sys_time.episode_ctr;

  if(cycle == 1 && episode == 1) print_header(*out);
  if(cycle == 1) print_episode_header(*out, episode);

  if(output_mode == 0) {
    if (cycle>1) {
      print_cycle_info(*out, current_sys_time);
      write_om0(*out, prev_observation,  prev_action, current_state_vars.observed_state);
    }
  } else if(output_mode == 1) {
    print_cycle_info(*out, current_sys_time);
    write_om1_last(*out, current_state_vars , terminal , terminal_rating);
  } else {
    CLSERR("Unknown output mode! " << output_mode );
  }

  *out << std::flush;
}

void DefaultOutput::deinit() {
  if (out!=0) {*out<< std::flush;  delete out; out=0;}
  delete[] prev_observation;
  delete[] prev_action;
}

bool DefaultOutput::init(const sys_dim_struct  &sys_dim_info,
                  const sys_time_struct &sys_time_info,
                  const char            *fname) {
  // values
  plant_state_dim      = sys_dim_info.plant_state_dim;
  action_dim           = sys_dim_info.action_dim;
  observation_dim      = sys_dim_info.observed_state_dim;
  external_signal_dim  = sys_dim_info.external_signal_dim;
  measurement_dim      = sys_dim_info.measurement_dim;
  delta_t              = sys_time_info.delta_t;

  //defaults
  protocol_freq        = 1;
  out                  = 0;
  sprintf(ProtFName,"%s","default.prot");
  output_mode          = -1; // no output
  outp_width           = 10;
  outp_prec            = 5;
  prev_observation     = new double[observation_dim];
  prev_action          = new double[action_dim];

  // default: write all observation dims
  selected_observation_ids.clear();
  for (int i=0; i<observation_dim; i++) selected_observation_ids.push_back(i);
  // default: write all action dims
  selected_action_ids.clear();
  for (int i=0; i<action_dim; i++) selected_action_ids.push_back(i);
  // default: write no plant_state, external_signal and measurement information
  selected_plant_state_ids.clear();
  selected_external_signal_ids.clear();
  selected_measurement_ids.clear();

  if (!read_options(fname)) {
    EOUT("Reading option failed!");
    return false;
  }

  if(output_mode >=0) {
    out = new ofstream(ProtFName);
    if (!out || !*out) {
      EOUT("Can't open file: (" << ProtFName << ") for writing!");
      return false;
    }
  }

  gettimeofday(&start_tval,NULL);

  return true;
}

bool DefaultOutput::read_options(const char * fname) 
{
  char paramstr[MAX_STR_LEN];

  if(fname == 0)
    return true;

  ValueParser vp(fname,"Output");

  if(vp.get("output_mode",paramstr,MAX_STR_LEN)>=0){
    if(strcmp(paramstr,"none")== 0)
      output_mode = -1;
    else if(strcmp(paramstr,"standard")== 0)
      output_mode = 1;
    else if(strcmp(paramstr,"xux")== 0)
      output_mode = 0;
    else
      EOUT("Unknown input mode: (" << paramstr <<") using default (standard).");
    // default: -1 (none)
  }

  vp.get("output_file",ProtFName,MAX_STR_LEN);

  int tmp[200];
  int n;
  n=vp.get("observation_ids" , tmp, 200);
  if(n >= 0){
    selected_observation_ids.clear();
    for(int i=0; i<n;i++) {
      if (tmp[i] >= observation_dim || tmp[i] < 0) {
        EOUT("observation_ids " << tmp[i] << " out ouf bounds!");
        return false;
      }
      selected_observation_ids.push_back(tmp[i]);
    }
  }
  n=vp.get("action_ids" , tmp, 200);
  if(n >= 0){
    selected_action_ids.clear();
    for(int i=0; i<n;i++) {
      if (tmp[i] >= action_dim || tmp[i] < 0) {
        EOUT("action_ids " << tmp[i] << " out ouf bounds!");
        return false;
      }
      selected_action_ids.push_back(tmp[i]);
    }
  }
  n=vp.get("plant_state_ids" , tmp, 200);
  if(n >= 0){
    selected_plant_state_ids.clear();
    for(int i=0; i<n;i++) {
      if (tmp[i] >= plant_state_dim || tmp[i] < 0) {
        EOUT("plant_state_ids " << tmp[i] << " out ouf bounds!");
        return false;
      }
      selected_plant_state_ids.push_back(tmp[i]);
    }
  }
  n=vp.get("external_signal_ids" , tmp, 200);
  if(n >= 0){
    selected_external_signal_ids.clear();
    for(int i=0; i<n;i++) {
      if (tmp[i] >= external_signal_dim || tmp[i] < 0) {
        EOUT("external_signal_ids " << tmp[i] << " out ouf bounds!");
        return false;
      }
      selected_external_signal_ids.push_back(tmp[i]);
    }
  }
  n=vp.get("measurement_ids" , tmp, 200);
  if(n >= 0){
    this->selected_measurement_ids.clear();
    for(int i=0; i<n;i++) {
      if (tmp[i] >= this->measurement_dim || tmp[i] < 0) {
        EOUT("measurement_ids " << tmp[i] << " out ouf bounds!");
        return false;
      }
      this->selected_measurement_ids.push_back(tmp[i]);
    }
  }


  return true;
}

REGISTER_OUTPUT(DefaultOutput, "Writes transition data to file.")

