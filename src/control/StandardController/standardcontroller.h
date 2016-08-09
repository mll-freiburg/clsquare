/*
clsquare - closed loop simulation system
Copyright (c) 2004, Neuroinformatics Group, Prof. Dr. Martin Riedmiller,
University of Osnabrueck

All rights reserved.

Author: Roland Hafner

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

#ifndef _STANDARDCONTROLLER_H_
#define _STANDARDCONTROLLER_H_

#include "controller.h"
#include "setdef.h"
#include "global.h"
#include "funcgen1d.h"

/** A controller that implements various standard techniques for control and test.
  * \attention At the moment the controller is only applicable to plants with
  *   one-dimensional actions! You can use the MixControl meta-controller
  *   to build a multi-dimensional action from several of these modules, however.
  *
  * The control behavior can be changed by the \b controller_mode.
  * Available controller modes are:
  * \li  0 : set the default action (constant).
  * \li  2 : user controlled input (extern gui); requires the parameter
  *          \b gui_request_string to be specified. An example for such a
  *          string would be "SLIDER Aktion -1 1 0 100", which requests
  *          a slider labeled "Aktion" with values ranging between -1 and 1,
  *          100 increments, and a starting value of 0.
  * \li 10 : PID control (nyi); requires the parameters \b kp, \b ki, \kd
  *          and \b error_id, the last of which is the state index in relation
  *          to which the control signal should be generated.
  *
  * The following universal parameters are also available:
  * \li \b xwork: the working range of the controller; will abort
  *     the episode if left
  * \li \b max_action, \b min_action: action range, applied after calculation
  * \li \b default_action: action used if controller_mode is 0 or invalid
  *
  * @author Roland Hafner
  * @ingroup CONTROLLER   
  * @ingroup STATIC
  */
class StandardController : public Controller {
 public:
  virtual ~StandardController(){;}
  virtual bool get_action(const double* observed_state, double* action);
  virtual bool init(const int observation_dim, const int action_dim, double deltat, const char* fname=0, const char* chapter=0);
  virtual void deinit();
  virtual void get_help( std::ostream& out );

  virtual void notify_episode_stops(const double* current_observed_state);
  
 protected:
  int       xdim;
  int       udim;
  double    delta_t;

  int       controller_mode;

  int       episode_step_counter;
  long      episode_counter;

  double    max_action;
  double    min_action;
  double    default_action;

  SetDef    xwork;

  bool      read_options(const char* fname, const char* chapter);

  RefGUIClient*     ref_gui_client;

  int               error_id;
  double            kp;
  double            ki;
  double            kd;

  double            last_err;

};
#endif
