/*
clsquare - closed loop simulation system
Copyright (c) 2010-2012 Machine Learning Lab, 
Prof. Dr. Martin Riedmiller, University of Freiburg

Author: Thomas Lampe

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

#ifndef _MIXCONTROLOPS_H_
#define _MIXCONTROLOPS_H_

#include "controller.h"
#include "mixcontrol.h"
#include "setdef.h"
#include <vector>

/** MixControl operation to choose an action based on the current state.
  * Basic parameters are the same as for every MixControlOperation.
  * The parameter \b controller_0 describes the index of the default controller, \b controller_1 is the one chosen
  * under certain conditions. The secondary controller will be activated as soon as the state is within any of the
  * StateDef sets described by the config parameters \b activate_x, where \e x is a continuous index starting at zero.
  * The controller supports hysteresis; once the secondary action has been activated, it will be used until any
  * condition described by the parameters \b release_x is met. If no release condition is specified, the default
  * action will be resumed as soon as the activation criteria are not met anymore. If activation and release areas
  * overlap, the default action is preferred.
  *
  * @ingroup CONTROLLER
  * @ingroup META
  * @author Thomas Lampe
  **/
class MixControlCondition : public MixControlOperation {
  public:
	  bool get_action (const double* state, double* action);
    bool init (const std::vector<MixControlSlave> *slaves, const char* fname, const char* chapter, const int id);
    inline void notify_episode_starts () {_choice=0;};
	  MixControlCondition() {_slavenum=2;};
	  ~MixControlCondition() {};
  protected:
    std::vector<SetDef> _activate, _release;
    int _i, _choice;
    bool _never_release, _default_release;
};

/** MixControl operation to compute basic arithmetics on two actions.
  * Basic parameters are the same as for every MixControlOperation.
  * The operation is specified by the parameter \b operator, which is a single character corresponding to it.
  * Possible values are \e +, \e -, \e *, \e / and \e ^.
  *
  * @ingroup CONTROLLER
  * @ingroup META
  * @author Thomas Lampe
  **/
class MixControlCombine : public MixControlOperation {
  public:
    bool get_action (const double* state, double* action);
    bool init (const std::vector<MixControlSlave> *slaves, const char* fname, const char* chapter, const int id);
    MixControlCombine() {_slavenum=2;};
    ~MixControlCombine() {};
  protected:
    double _mode[2];
    bool _pow;
};

/** MixControl operation to compute a weighted sum of two actions.
  * Basic parameters are the same as for every MixControlOperation.
  * The weight between both both controllers is specified by the parameter <b> factors = <double>+ </b>, which
  * contains as many values as there are action dimensions (specified by the plant or by \e MixControl::action_dim_x).
  * The sum is then computed for each dimension \e i as:
  *
  * <em> action[i] = factors[i] * action_0[i] + (1 - factors[i]) * action_1[i] </em>
  *
  * @ingroup CONTROLLER
  * @ingroup META
  * @author Thomas Lampe
  **/
class MixControlWeight : public MixControlOperation {
  public:
    bool get_action (const double* state, double* action);
    bool init (const std::vector<MixControlSlave> *slaves, const char* fname, const char* chapter, const int id);
    MixControlWeight() {_slavenum=2;};
    ~MixControlWeight() {};
  protected:
    double *_factor;
};

/** MixControl operation to select a subcontroller based on
  * input from CLSquare's pipe.
  * Supports an arbitrary number of sub-controllers. Each must have
  * an extra parameter \b command_i, which gives the pipe content
  * needed to activate the controller. All commands need to be
  * preceded by the string "controller_cmd".
  * \note Note that since the pipe is checked after the controller has
  *       generated an action but before the action is performed by the
  *       plant, the controller switch will only take effect in the
  *       following cycle after it has been detected.
  *
  * @ingroup CONTROLLER
  * @ingroup META
  * @author Thomas Lampe
  **/
class MixControlPipe : public MixControlOperation {
  public:
    bool get_action (const double* state, double* action);
    bool init (const std::vector<MixControlSlave> *slaves, const char* fname, const char* chapter, const int id);
    void notify_command_string (const char* buf);
    void notify_episode_starts ();
    MixControlPipe() {_slavenum=-1;};
    ~MixControlPipe() {};
  protected:
    std::vector<std::string> _commands;
    int _current;
};

class MixControlToggle : public MixControlOperation {
  public:
    bool get_action (const double* state, double* action);
    bool init (const std::vector<MixControlSlave> *slaves, const char* fname, const char* chapter, const int id);
    void notify_episode_starts ();
    MixControlToggle() {_slavenum=-1;};
    ~MixControlToggle() {};
  protected:
    SetDef _tset;
    int _current;
    bool _toggled, _rewind;
};

#endif
