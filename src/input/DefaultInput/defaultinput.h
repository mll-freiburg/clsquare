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

#ifndef _DEFAULT_INPUT_H_
#define _DEFAULT_INPUT_H_

#include "input.h"
#include "setdef.h"
#include "funcgen1d.h"
#include <vector>

/** Default implementation for an input module with range- and file-based modes.
  * Supports several basic input modes, which are toggled by the parameter
  * \b input_mode.
  * \li \e from_file: assumes that the file given by \b input_file contains a
  *     list of initial states, with entries being separated by spaces.
  *     States from this file will be used in serial order by default, or
  *     in random order if \b order_of_presentation has been set to \e random.
  * \li \e random: randomly chooses initial states from the SetDef range
  *     specified by \b xinit. Dimensions left blank in xinit will be filled
  *     with NaN. The input range can be increased over time through
  *     the parameters \b growth_freq and \b growth_step, the latter of which
  *     is also a SetDef, specifying the step size in either direction. The
  *     maximum range can be specified by \b growth_limit.
  * \li \e none: simply returns a list of NaN.
  *
  * If the parameter \b protfile has been defined, the input states that
  * were chosen each episode are logged in the file it refers to.
  *
  * The input module also provides a mechanism for generating reference
  * inputs for plants that require such. Unlike initial states, reference
  * inputs may change throughout the episode. The frequency at which they
  * are chosen is determined by the parameter \b ref_inp_change_freq, and
  * the number of changes can be limited by \b ref_inp_change_num. If
  * \b ref_inp_integrating is set to \e true, new values are added to the
  * old ones rather than replacing them.
  *
  * The method of generating the reference input is determined on a per-
  * dimension basis by the parameter \b ref_inp_modes, which is expected
  * to be a list of the same length as the plant's reference input
  * dimension.
  * \li 0: Uses a one-dimensional linear interpolation function of type
  *     InterpolLinFun1D. For each dimension \e i, a parameter
  *     \b ref_inp_fun_i must define the name of a file where the
  *     function is described.
  * \li 1: Randomly draws values from a SetDef range, which is defined
  *     for each dimension \e i by \b ref_inp_range_i.
  * \li 2: Uses a parameterized standard input function.
  *     For each dimension \e, expects a parametrization \b
  *     ref_inp_stdfun_params_i, which contains a list \f$p\f$ of
  *     length 5. Each reference input is then computed through the
  *     formula \f$r_1 sin ( \frac{ 2\pi ( c\Delta_t + r_3 ) }
  *     { r_0 } ) + r_2\f$ until the time \f$c\Delta_t \geq r_4\f$
  *     has elapsed, and 0 afterwards. If \f$r_4\f$ is negative,
  *     no timeout is used.
  * \li 3: Calculates the reference input through the same function
  *     as mode 2, but accepts multiple parametrizations for each
  *     dimensions. Parameters are defined as \b
  *     ref_inp_stdfun_params_i_h, with \e h being a continuous
  *     index starting at 0. The module cycles through these
  *     parameter sets, switching to then next one at the beginning
  *     of each episode.
  * \li 4: Gets values from a GUI interface via TCP/IP, such as the
  *     GraphicalUserInterface tool bundled with CLSquare. For each
  *     dimension \e i, a parameter \b ref_gui_i must define a
  *     valid request string of the form "SLIDER <label> <min> <max>
  *     <default> <steps>".
  *
  * @author Roland Hafner
  * @ingroup INPUT */
class DefaultInput {
 public:
  virtual void get_initial_episode_plant_state(double* initial_plant_state, const int episode);
  virtual void get_external_signal(double* external_signal, long cycle_ctr);
  virtual bool init(const int _plant_state_dim, const int _action_dim, const int _external_signal_dim, const double _delta_t, const char *fname=0);
  virtual void deinit();

 protected:
  int verbosity;

  /** counts times that get_initial_episode_plant_state was called.
      Note: originally described as counting episodes, but behavior does not match. */
  long int seq_counter;

  /** counts episodes. */
  long int seq_num;

  /** definition of working area. All random states are choosen out of the working area. */
  SetDef  XworkSet;

  /** order of presentation of states from file.
   * \li 0: serial
   * \li 1 random */   
  int  order_of_presentation; 

  /** list of initial states from file. */
  std::vector< std::vector<double> > stups;

  /** parses initial states from file. */
  bool parse_input_file(const char *fname);

  /** current index of starting state. */
  int     akt_stup_id;

  /** gets a randomly chosen starting state from the working area.
   * \param state initial state 
   * \param target_state randomly chosen target state */  
  void get_state_mode0(double* plant_state, bool grow);

  /** gets a state chosen from a list of states (file)
   * \param state initial state
   * \param target_state target state */
  void get_state_mode1(double* plant_state);
 
  /** dimension of state space. */
  int plant_state_dim;

  /** dimension of action space. */
  int action_dim;

  /** dimension of external signal **/
  int external_signal_dim;

  /** duration of one control cycle in seconds. */
  float delta_t;
  
  /** mode of choosing initial states and target states. */
  int input_mode;
  
  /** modes of choosing external signal variables. */
  int*    ref_inp_modes;
  double* last_ref_inp;

  int     ref_inp_change_freq;
  int     ref_inp_change_num;
  int     ref_input_changed;
  bool    ref_inp_change_integrating;

  /** rate at which the input ranges increase */
  int growth_freq;
  SetDef growthset;
  SetDef growthmax;
  

  /** reads configuration of input module. 
   * \param fname filename
   * \return true for success */
  bool read_options(const char * fname);

  /** protocol inputs activities */
  std::ofstream protfile;


  std::vector< InterpolLinFun1D* > ref_inp_fun;
  std::vector< RefGUIClient* > ref_gui_client;
  std::vector< double* > ref_inp_stdfun_params;
  // mode 3
  std::vector< std::vector< double* > > ref_inp_stdfun_params_list;
  std::vector< SetDef > ref_inp_range;
  
};

#endif

