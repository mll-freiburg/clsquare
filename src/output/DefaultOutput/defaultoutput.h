/*
clsquare - closed loop simulation system
Copyright (c) 2004, Neuroinformatics Group, Prof. Dr. Martin Riedmiller,
University of Osnabrueck

Author: Martin Riedmiller, Roland Hafner

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

#ifndef _DEFAULT_OUTPUT_H_
#define _DEFAULT_OUTPUT_H_

#include "output.h"
#include <sys/time.h>
#include <vector>
#include "kerneldatatypes.h"

/**
 * Default module for logging episodes in a file.
 * Provides several basic modes.
 *
 *  \section Configurable Parameters
 *  Following parameters are read in section [Output] of the configuration file:
 *  \li \c output_mode (string) none: no output; xux: transition mode (only observations and actions); standard: standard mode
 *  \li \c output_file (string) filename of the generated output file (an existing file will be replaced without notice)
 *
 *  Using following parameters, the behaviour of the module can be influenced in detail:
 *  \li \c observation_ids (list of int) ids of the observation vector entries to be written [0,observation_dim-1]; default: all observation entries
 *  \li \c action_ids (list of int) ids of the action vector entries to be written [0,action_dim-1]; default: all action entries
 *  \li \c plant_state_ids (list of int)
 *  \li \c external_signal_ids (list of int)
 *  \li \c measurement_ids (list of int)
 * @ingroup OUTPUT
 */
class DefaultOutput : public Output
{
public:
  void notify               ( const sys_state_variables&     current_state_vars,
                              const double*                  current_action,
                              const sys_time_struct&         current_sys_time,
                              double                         transition_rating);

  void notify_episode_stops ( const sys_state_variables&     current_state_vars,
                              const sys_time_struct&         current_sys_time,
                              bool                           terminal,
                              double                         terminal_rating);

  bool init(const sys_dim_struct& sys_dim_info , const sys_time_struct& sys_time_info, const char *fname=0);

  void deinit();

protected:
  void write_om0(std::ostream& _out, const double* _prev_observation, const double* _prev_action,  const double* _observation);

  void write_om1(std::ostream&                     _out,
                 const sys_state_variables&        _state_vars ,
                 const double*                     _action,
                 double                            _transition_rating);

  void write_om1_last(std::ostream&                _out,
                      const sys_state_variables&   _state_vars ,
                      bool                         _terminal,
                      double                       _terminal_rating);

  void print_observation      (std::ostream& _out, const double* _observation);
  void print_measurement      (std::ostream& _out, const double* _measurement);
  void print_action           (std::ostream& _out, const double* _action);
  void print_plant_state      (std::ostream& _out, const double* _plant_state);
  void print_external_signal  (std::ostream& _out, const double* _reference_input);
  void print_cycle_info       (std::ostream& _out, const sys_time_struct& sys_time_info);
  void print_header           (std::ostream& _out);
  void print_episode_header   (std::ostream& _out, long int _episode);

  int      plant_state_dim;
  int      action_dim;
  int      observation_dim;
  int      external_signal_dim;
  int      measurement_dim;
  double   delta_t;

  int      protocol_freq;

  std::vector< int > selected_plant_state_ids;
  std::vector< int > selected_observation_ids;
  std::vector< int > selected_action_ids;
  std::vector< int > selected_external_signal_ids;
  std::vector< int > selected_measurement_ids;

  timeval  start_tval;
  double  *prev_observation;
  double  *prev_action;

  std::ostream  *out;
  int  outp_width, outp_prec;
  char ProtFName[MAX_STR_LEN];
  int  output_mode;

  bool read_options(const char * fname);
};

#endif

