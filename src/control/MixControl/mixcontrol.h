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

#ifndef _MIXCONTROL_H_
#define _MIXCONTROL_H_

#include "controller.h"
#include "setdef.h"
#include <vector>

/**
  * MixControl composite action assignment.
  **/
struct MixControlAction
{
  double multi, add;
  int controller, action;
  SetDef condition;
};

/**
  * MixControl sub-controller specification.
  **/
struct MixControlSlave
{
  int sdim, adim, *input, _i;
  double *state, *action;
  char *name, *chapter;
  Controller *base;
  bool success;

  // create reduced temporary state
  void reduce_state (const double* system_state) {
    for (_i=0; _i<sdim; _i++)
      state[_i] = system_state[input[_i]];
  };
};

/**
  * Provides the basic interface for special MixControl Operations.
  * All operations share a few common parameters:
  *
  * Parameters of the form <b> controller_x = <int> </b>, with \e x being a continuous index starting at 0, describe sub-controllers
  * that the opertation is carried out with. The parameter is a single number, which refers to the respective sub-controller
  * specified by MixControl. For instance, <em> controller_0 = 2 </em> would cause the controller described by \e 
  * MixControl::controller_2 to be used as the first element of the operation. The number of subcontrollers to be thus specified
  * depends on the specific operation to be used.
  *
  * Optional parameters of the form <b> actions_x = <int>+ </b> may be used to re-order the action vector of a sub-controller. For
  * instance, <em> action_0 = 0 0 0 </em> and <em> action_1 = 2 4 5 </em> would cause action 0 of the primary controller to be
  * combined with actions 3, 4 and 5 of the secondary one. The length of the assignment is expected to be equal to the plant's
  * action dimensionality, or the the parameter \e MixControl::action_dim_x corresponding to the chosen controller.
  *
  * Note that operations are only usable when created from within MixControl, and cannot be used as an application's main
  * controller.
  **/
class MixControlOperation : public Controller {
  public:
    bool init (const int observed_state_dim, const int action_dim, double deltat, const char* fname=0, const char* chapter=0);
    /** Second initialization method that is called once all controllers have been initialized.
      * This method should call MixControlOperation::check_params to maintain a reference to
      * the other modules and chech constraints. */
    virtual bool init (const std::vector<MixControlSlave> *slaves, const char* fname, const char* chapter, const int id) = 0;
  protected:
    bool check_params (const std::vector<MixControlSlave> *slaves, const int id);
    int _i, _adim, _sdim, **_acts, _slavenum, *_cons;
    const std::vector<MixControlSlave> *_slaves;
    bool _init;
};

/**
  * Provides a meta-controller that creates instances of other controller types and assigns them to
  * different dimensions of the action, thus allowing to mix various types of controllers.
  *
  * A sub-controller is added through a config entry of the form <em>controller_i = ControllerType Section</em>,
  * where \e i begins at 0 for the first controller and is numbered incrementally for additional ones.
  * Specifying the config section where the controller is defined is optional; [Controller_i] is used by default.
  *
  * In addition, it is possible to define the state that each controller receives and the action it generates.
  * The parameter \e action_dim_i sets the dimensionality of the action to a value different from the one
  * defined by the plant. Likewise, \e state_dim_i does the same for the state; \e state_assign then determines
  * which dimensions of the plant measurement should be passed on to the controller.
  *
  * The actions that are generated by the sub-controllers then have to be combined back into an action to be
  * sent to the plant, which is controlled through the parameter \e assignment. It consists of pairs of a 
  * sub-controller and action index, one for each of the plant action dimensions. For instance,
  * <em>assignment = [1 3] [0 1]</em> would use action 3 of controller 1 for the first plant action, and action 1
  * of controller 0 for the second.
  *
  * Lastly, two more parameters shape the final action:
  * \li \e multi: defines multipliers that will be applied element-wise to the composite action
  * \li \e add: defines constant values that will be added to the composite action (after the multiplication step)
  *
  * <em> (Note: the following method for specifying conditions is deprecated. For new applications, use the
  *  controller type MixControlCondition instead.) </em>
  *
  * It is also possible to use different controllers for one action dimension, with the eligible controller
  * being determined based on the current system measurement. An alternative \e i for an action \e x can
  * be specified by adding the two config entries, where \e i is the index of the alternative, starting at 0.
  * If two different alternatives would match a given state, the one with the lower number will be chosen, hence
  * they should be ordered from most specific to most general. If no alternative matches, the default controller
  * as given by \e assignment will be used.
  * \li \e action_x_i_params: provides 4 numbers that specify, in this order, the index of the controller
  *   to be used, the action index of of the controller, the multiplier and the addition constant.
  * \li \e action_x_i_condition: provides a state definition as per the SetDef syntax which describes the
  *   set of states in which the alternative will apply.
  *
  * For instance, <em>action_0_0_params = 1 3 2 0</em> and <em>action_0_0_condition = [0.5 1][]</em> would
  * use the double value of action dimension 3 of controller 1 whenever dimension 0 of the measurement is
  * betweem 0.5 and 1.
  *
  * @ingroup CONTROLLER
  * @ingroup META
  * @author Thomas Lampe
  **/
class MixControl : public Controller {
  public:
	  bool get_action (const double* state, double* action);
	  bool init (const int observed_state_dim, const int action_dim, double deltat, const char* fname=0, const char* chapter=0);
	  void deinit ();
    void notify_episode_starts ();
    void notify_episode_stops (const double* current_observed_state);
    void notify_transition (const double* observed_state, const double* action, const double* next_observed_state, const double reward, const bool is_terminal_state, const double terminal_reward);
    void notify_command_string (const char* buf);
    bool check_initial_state (const double* initial_observed_state, const int observation_dim);

	  MixControl() {};
	  ~MixControl() {};

  protected:
    bool _success;
    unsigned int _i, _j, _k, _state_dim;
    double *_tmp_observation;
    std::vector<std::vector<MixControlAction> > _assignment;
    std::vector<MixControlSlave> _slaves;
};

#endif
