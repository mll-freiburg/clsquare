/*
clsquare - closed loop simulation system
Copyright (c) 2004, Neuroinformatics Group, Prof. Dr. Martin Riedmiller,
University of Osnabrueck

Author: Roland Hafner
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
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include "valueparser.h"
#include "str2val.h"
#include "defaultinput.h"
#include <fstream>
#include "global.h"
#include <cmath>

#define PROT_MSG(_msg) if(protfile.is_open() == true) protfile<<_msg


void DefaultInput::get_state_mode0(double* initial_plant_state, bool grow)
{
  int h = XworkSet.numSubsets();
 
  if (grow)
    for (int s=0; s<h; s++) {
      std::cout << "Input set #" << s << " grows to ";
      for (int i=0; i<plant_state_dim; i++) {
        XworkSet.subsets[s][i].max += growthset.subsets[0][i].max;
        if (XworkSet.subsets[s][i].max > growthmax.subsets[0][i].max) XworkSet.subsets[s][i].max = growthmax.subsets[0][i].max;
        XworkSet.subsets[s][i].min += growthset.subsets[0][i].min;
        if (XworkSet.subsets[s][i].min < growthmax.subsets[0][i].min) XworkSet.subsets[s][i].min = growthmax.subsets[0][i].min;
        std::cout << "[" << XworkSet.subsets[s][i].min << " " << XworkSet.subsets[s][i].max << "] ";
      }
      std::cout << std::endl;
    }

  if (h<=0) EOUT("No subsets for xinit defined!");
  if (h>1) h=(int) (h * drand48()); // choose one subset randomly
  else (h-=1);
  for (int i=0; i<plant_state_dim; i++) {
    double range  = XworkSet.subsets[h][i].max - XworkSet.subsets[h][i].min;
    double offset = XworkSet.subsets[h][i].min;
    initial_plant_state[i] = offset;
    if (range== 0) continue;
    initial_plant_state[i] += range * drand48();
  }
}

void DefaultInput::get_state_mode1(double* initial_plant_state)
{
  if (stups.size()<=0) {
    EOUT("No startup positions loaded from file!");
    return;
  }

  double r=0;
  int tmp_id;

  switch (order_of_presentation) {
  case (0) :
    tmp_id = akt_stup_id;
    akt_stup_id+=1;
    if (akt_stup_id >= (int)stups.size()) akt_stup_id=0;
    break;
  case (1) :
    r = drand48();
    tmp_id = (int) (r * stups.size());
    break;
  default:
    EOUT("No such order of presentation: (" <<  order_of_presentation << ")!");
    return;
  }

  for (int i=0; i<plant_state_dim; i++) {
    initial_plant_state[i]=stups[tmp_id][i];
  }
}

/*****************************************************************
 * DefaultInput: set new state
 *****************************************************************/

void DefaultInput::get_initial_episode_plant_state(double* initial_plant_state, const int episode)
{
  seq_counter++;

  switch (input_mode) {
  case -1:
    break;
  case 0:
    get_state_mode0(initial_plant_state, growth_freq>0 && episode>1 && seq_num!=episode && episode%growth_freq==0);
    break;
  case 1:
    get_state_mode1(initial_plant_state);
    break;
  default:
    EOUT("Unknown input_mode: (" << input_mode << ")!");
    CLSERR("Unknown input_mode!");
  }
  ref_input_changed=0;
  PROT_MSG("# initial state at episode "<<seq_counter<<endl);
  for (int i=0; i<plant_state_dim; i++)
    PROT_MSG(initial_plant_state[i]<<" ");
  PROT_MSG(endl);
  seq_num = episode;
}


void DefaultInput::get_external_signal(double* reference_input, long cycle_ctr)
{
  for (int i=0; i<external_signal_dim; i++) {
    switch (ref_inp_modes[i]) {
    case 0:
      if ( ref_inp_fun[i] == 0)
        reference_input[i]=0.0;
      else
        reference_input[i] = ref_inp_fun[i]->get( cycle_ctr * delta_t );
      break;
    case 1:
      if (cycle_ctr == 1
          || (ref_inp_change_freq > 0 && cycle_ctr%ref_inp_change_freq == 0 &&  ref_input_changed <=  ref_inp_change_num)) {
        int h = ref_inp_range[i].numSubsets();
        if (h<=0) EOUT("No subsets for ref_inp_range" << i << " defined!");
        if (h>1) h=(int) (h * drand48()); // choosre one subset randomly
        else (h-=1);

        double range  = ref_inp_range[i].subsets[h][0].max - ref_inp_range[i].subsets[h][0].min;
        double offset = ref_inp_range[i].subsets[h][0].min;
        double tmp = last_ref_inp[i];
        if ( cycle_ctr == 1 ) tmp=0;

        last_ref_inp[i] = offset;
        if (range!= 0)
          last_ref_inp[i] += range * drand48();

        if ( ref_inp_change_integrating ) last_ref_inp[i] += tmp;

        ref_input_changed++;
      }

      reference_input[i] = last_ref_inp[i];
      break;
    case 2:
      // sin
      if ( (ref_inp_stdfun_params[i][4] > 0) && ((cycle_ctr* delta_t) >= ref_inp_stdfun_params[i][4]) ) {
        reference_input[i] = 0.0;
      } else {
        reference_input[i] = sin(
                               ( (cycle_ctr * delta_t)  + ref_inp_stdfun_params[i][3] )
                               * (1.0*(2*M_PI))/ ref_inp_stdfun_params[i][0] )
                             * ref_inp_stdfun_params[i][1]
                             + ref_inp_stdfun_params[i][2];
      }
      break;
    case 3: {
      reference_input[i] = 0.0;
      int h = seq_counter % (int)ref_inp_stdfun_params_list[i].size();
      if ( (ref_inp_stdfun_params_list[i][h][4] > 0) && ((cycle_ctr* delta_t) >= ref_inp_stdfun_params_list[i][h][4]) ) {
        reference_input[i] += 0.0;
      } else {
        reference_input[i] += sin(
                                ( (cycle_ctr * delta_t)  + ref_inp_stdfun_params_list[i][h][3] )
                                * (1.0*(2*M_PI))/ ref_inp_stdfun_params_list[i][h][0] )
                              * ref_inp_stdfun_params_list[i][h][1]
                              + ref_inp_stdfun_params_list[i][h][2];
      }

      //}
    }
    break;
    case 4:
      reference_input[i] = ref_gui_client[i]->get(cycle_ctr * delta_t);
      break;
    }
    last_ref_inp[i]=reference_input[i];
  }
}


void DefaultInput::deinit()
{
  for (int i=0; i<external_signal_dim; i++) {
    if (ref_inp_fun[i]!=0) delete ref_inp_fun[i];
    if (ref_gui_client[i]!=0) delete ref_gui_client[i];
  }
  delete [] last_ref_inp;
  delete [] ref_inp_modes;
  if (protfile)
    protfile.close();
}

bool DefaultInput::init(const int _plant_state_dim, const int _action_dim, const int _reference_input_dim, double _delta_t, const char *fname)
{
  plant_state_dim       = _plant_state_dim;
  action_dim            = _action_dim;
  external_signal_dim   = _reference_input_dim;
  delta_t               = _delta_t;
  seq_counter           = 0;
  seq_num               = 0;
  verbosity             = 0;

  //defaults
  input_mode            = 0; //< random from interval
  order_of_presentation = 0; //< for mode 1 only

  ref_inp_fun.resize(_reference_input_dim);
  ref_gui_client.resize(_reference_input_dim);
  ref_inp_stdfun_params.resize(_reference_input_dim);

  ref_inp_stdfun_params_list.resize(_reference_input_dim);

  ref_inp_range.resize(_reference_input_dim);
  ref_inp_modes = new int[_reference_input_dim];
  last_ref_inp  = new double[_reference_input_dim];
  ref_inp_change_freq = -1;

  for (int i=0; i<_reference_input_dim; i++) {
    ref_inp_fun[i]   = 0;
    ref_gui_client[i] = 0;
    ref_inp_modes[i]  = 0;
    last_ref_inp[i]   = 0;
    ref_inp_stdfun_params[i]=new double[5];
    for (int h=0; h<5; h++) ref_inp_stdfun_params[i][h]=0.0;
  }

  ref_input_changed  = 0;
  ref_inp_change_num = 10000;
  ref_inp_change_integrating = false;

  if (read_options(fname) == false)
    return false;

  akt_stup_id = 0;


  return true;
}

bool DefaultInput::read_options(const char * fname)
{
  char paramstr[MAX_STR_LEN];
  char tmp_str[MAX_STR_LEN];

  if (fname == 0)
    return true;

  ValueParser vp(fname,"Input");

  vp.get("verbosity", verbosity);

  if (vp.get("protfile",tmp_str,MAX_STR_LEN)>=0) {
    protfile.open(tmp_str);
    if (!protfile) {
      EOUT("Error: Cannot open protocol file"<<endl);
      return false;
    }
  }


  if (vp.get("input_mode",paramstr,MAX_STR_LEN)>=0) {
    if (strcmp(paramstr,"from_file")== 0)
      input_mode = 1;
    else if (strcmp(paramstr,"random")== 0)
      input_mode = 0;
    else if (strcmp(paramstr,"none")== 0)
      input_mode = -1;
    else
      EOUT("Unknown input mode: (" << paramstr << ") using default (random)!");
  }

  if (input_mode == -1)
    return true;

  if (input_mode == 0) {
    if (vp.get("xinit",paramstr,MAX_STR_LEN)>=0) {
      XworkSet.parseStr(paramstr, plant_state_dim);
    } else {
      EOUT("Missing Parameter: (xinit) in cfg file section [Input]!)");
      return false;
    }

    // range growth
    vp.get("growth_freq", growth_freq, 0);
    if (growth_freq > 0) {
      if (vp.get("growth_step", paramstr, MAX_STR_LEN) > 0) {
        growthset.parseStr(paramstr, plant_state_dim);
        if (growthset.subsets[0].size() < (unsigned int)plant_state_dim) {
          EOUT("Insufficient number of growth steps specified.");
          return false;
        }
      } else {
        EOUT("Missing Parameter: (growth_step) in cfg file section [Input]!)");
        return false;
      }

      if (vp.get("growth_limit", paramstr, MAX_STR_LEN) > 0) {
        growthmax.parseStr(paramstr, plant_state_dim);
        if (growthmax.subsets[0].size() < (unsigned int)plant_state_dim) {
          EOUT("Insufficient number of growth limits specified.");
          return false;
        }
      } else {
        EOUT("Missing Parameter: (growth_limit) in cfg file section [Input]!)");
        return false;
      }
    }

  } else if (input_mode == 1) {
    if (vp.get("input_file",paramstr,MAX_STR_LEN)>=0) {
      if (!parse_input_file(paramstr)) return false;
    } else {
      EOUT("Missing Parameter (input_file) in cfg file section [Input]!");
      return false;
    }
    if (vp.get("order_of_presentation",paramstr,MAX_STR_LEN)>=0) {
      if (strcmp(paramstr,"random")== 0)
        order_of_presentation = 1;
      else if (strcmp(paramstr,"serial")== 0)
        order_of_presentation = 0;
      else {
        EOUT("Unknown (order_of_presentation) value: " << paramstr << ") using default (serial)!");
        order_of_presentation = 0;
        // default: 0 (serial)
      }
    }
  } else {
    EOUT("Internal!! Unknown input parameter " << input_mode);
    return false;
  }

  vp.get("ref_inp_change_freq" , ref_inp_change_freq );
  vp.get("ref_inp_change_num"  , ref_inp_change_num );
  vp.get("ref_inp_change_integrating"  , ref_inp_change_integrating );

  if (external_signal_dim>0) {

    if (vp.get("ref_inp_modes" , ref_inp_modes , external_signal_dim) != external_signal_dim) {
      EOUT("ref_inp_modes of wrong dimension: ");
      return false;
    }

    for (int i=0; i<external_signal_dim; i++) {

      switch (ref_inp_modes[i]) {
      case 0: {
        std::stringstream pname;
        pname << "ref_inp_fun_" << i;
        if (vp.get(pname.str().c_str(),paramstr,MAX_STR_LEN)>=0) {
          ref_inp_fun[i] = new InterpolLinFun1D();
          if (!ref_inp_fun[i]->load(paramstr)) return false;
        }
      }
      break;
      case 1: {
        bool res = false;
        std::stringstream pname;
        pname << "ref_inp_range_" << i;
        if (vp.get(pname.str().c_str(),paramstr,MAX_STR_LEN)>=0) {
          res |= ref_inp_range[i].parseStr(paramstr, 1);
        }
        if (!res) {
          EOUT("Can not parse: " << pname.str());
          return false;
        }
      }
      break;
      case 2: {
        std::stringstream pname;
        pname << "ref_inp_stdfun_params_" << i;
        if (vp.get(pname.str().c_str(), ref_inp_stdfun_params[i] , 5 )<4) {
          EOUT("Can not parse: " << pname.str());
          return false;
        }
      }
      break;
      case 3: {
        bool res = true;
        int  h=0;
        do {
          std::stringstream pname;
          pname << "ref_inp_stdfun_params_" << i << "_" << h;
          double* p=new double[5];
          if (vp.get(pname.str().c_str(), p , 5 )<4) {
            res = false;
            if (h==0) {
              EOUT("Can not parse: " << pname.str());
              return false;
            }
          } else {
            ref_inp_stdfun_params_list[i].push_back( p );
          }
          h++;
        } while (res);

      }
      break;
      case 4: {
        std::stringstream pname;
        pname << "ref_gui_" << i;
        if (vp.get(pname.str().c_str(),paramstr,MAX_STR_LEN)>=0) {
          ref_gui_client[i] = new RefGUIClient();
          if (!ref_gui_client[i]->request_gui( paramstr )) return false;
        }
      }
      break;
      default:
        EOUT("No such ref_inp_mode: " << ref_inp_modes[i]);
        return false;
      }
    }
  }

  return true;
}

//void DefaultInput::get_help(std::ostream& out)
//{
//    out
//            << "General input module functions based on parameter setting:\n"
//            << "verbosity \t int \t the verbosity of the input module\n"
//            << "input_mode \t string \t the mode of initial plant state selection\n"
//            << "\t none \t dummy input module (initial plant state is 0\n"
//            << "\t random \t random selection based on SetDef xinit\n"
//            << "\t \t xinit \t SetDef \t range of input selection\n"
//            << "\t from_file \t selection based on states in a file (one state per line)\n"
//            << "\t \t input_file \t string \t filename\n"
//            << "\t \t order_of_presentation \t string \t how to sweep through the file\n"
//            << "\t \t \t random \t random selection\n"
//            << "\t \t \t serial \t serial selection\n"
//            ;
//}

bool DefaultInput::parse_input_file(const char *fname)
{
  std::istream *fin= 0;
  fin = new std::ifstream(fname);

  if (!fin || !(*fin)) {
    EOUT("Can't open file: " << fname);
    return false;
  }

  char  dum;
  double fdum;
  while (!(fin->eof())) {
    // skipp white spaces
    (*fin) >> std::ws;

    fin->get(dum);
    if (!(dum == '#' || dum == '\n')) {
      std::vector<double> stup;
      fin->unget();
      for (int i=0; i<plant_state_dim; i++) {
        if ((*fin) >> fdum)
          stup.push_back(fdum);
      }
      if (((int) stup.size()) == plant_state_dim)
        stups.push_back(stup);
      else {
        EOUT("Error on parsing input file " << fname << "not enough data in line.");
        continue;
      }
    }

    // skip rest of line
    do {
      fin->get(dum);
    } while ( (dum != '\n') && (!fin->eof()) );
  }
  if (fin) {
    delete fin;
  }

  return true;
}

REGISTER_INPUT(DefaultInput, "Default input implementation.")
