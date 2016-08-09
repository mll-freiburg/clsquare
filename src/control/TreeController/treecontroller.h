/*
clsquare - closed loop simulation system
Copyright (c) 2010, 2011 Machine Learning Lab, 
Prof. Dr. Martin Riedmiller, University of Freiburg

Author: Manuel Blum

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
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
*/ 

#ifndef __TREECONTROLLER_H__
#define __TREECONTROLLER_H__

#include "controller.h"
#include "aset.h"
#include "setdef.h"
#include "extratree.h"

#include <vector>
#include <string>

/** Implements a controller using fitted Q iteration with extra trees. 
 *  Config file parameters used: <br>
 *  <table>
 *  <tr><td>actions</td><td>whitespace separated list of discrete actions 
 *  </td><td>mandatory</td></tr><tr><td>update_freq</td><td> number of 
 *  episodes after which training starts<br>
 *  zero if you don't want to do learning <br>(make sure to provied 
 *  pretrained approximator in that case)</td><td> default = 1</td></tr>
 *  <tr><td>gamma </td><td>discount factor </td><td>default = 0.95</td></tr>
 *  <tr><td>epsilon </td><td>parameter for greedy exploration </td>
 *  <td>default = 0.5</td></tr>
 *  <tr><td>fq_cycles </td><td>number of fitted Q iterations </td>
 *  <td>default = 20</td></tr><tr><td>fileprefix </td>
 *  <td>ensembles will be written to and loaded from fileprefix0001 ... 
 *  </td><td>mandatory</td></tr>
 *  <tr><td>num_trees </td><td>number of trees in the ensemble </td>
 *  <td>default = 20</td></tr>
 *  <tr><td>num_tests </td><td>number of dimensions to check when splitting</td>
 *  <td> default = observation_dim</td></tr>
 *  <tr><td>n_min </td><td>minimum number of samples in a node <br>
 *  controls the depth of the resulting trees</td>
 *  <td>default = 10</td></tr>
 *  </table> 
 *  @author Manuel Blum
 *  @ingroup CONTROLLER
 *  @ingroup LEARNING
 */
class TreeController : public Controller
{
public:
	/** called by the kernel in order to get a new action from the controller.
	 \param observed_state Vector of doubles representing the present
	        observation. Its dimension matches the observation_dim given to the
	        init method.
	 \param action Vector of doubles that should be filled with the selected
	        action. The content of this vector is undefined and should be
	        ignored by the controller. Its dimension matches the action_dim
	        given to the init method.
	 \return false, if the controller wants to stop the present episode. */
	bool get_action(const double* observed_state, double* action);

	/** called by the kernel during start-up in order initialize the controller.
	 The GaussianProcessController checks for an action_dim of 1, as its the only
	 size of actions it supports. Furthermore, it stores the observation_dim
	 and reads controller-specific options from the config file fname under
	 section chapter.
	 \param observation_dim the dimension of the observations that the present
	        plant produces
	 \param action_dim the dimension of the actions the present plant produces.
	        must be one for the GaussianProcessController to work.
	 \param deltat the size of the time steps in seconds.
	 \param fname name of the config-file to use. This is specified by the user
	        when starting CLSquare from the command line.
	 \param chapter the section in the config file where the controller should
	        look for parameters. The default section is "Controller". */
	bool init(const int observation_dim, const int action_dim, double deltat,
	          const char* fname=0, const char* chapter=0);

	/** called by the kernel during tear-down to let the controller clean its
	 data structures. */
	void deinit();

	/** called by the kernel after every single transition that has been
	 exectued by the plant. This should be used to collect training data
	 for the learning algorithm.
	 \param observed_state beginning observation of the transition
	 \param action the action that has been executed
	 \param next_observed_state observation after executing the action
	 \param reward immediate reward 
	 \param is_terminal_state true if the transition has ended in an
	        absorbing terminal state 
  	 \param terminal_reward terminal reward*/
	void notify_transition(const double* observed_state,
			               const double* action,
			               const double* next_observed_state,
			               const double reward, 
			               const bool is_terminal_state, 
			               const double terminal_reward);

	/** called by the kernel whenever a episode has ended. */
	void notify_episode_stops(const double* current_observed_state);

//friend class TreeControllerWorkerData;
friend void* parallel_compute_target(void* arg);
private:
	
	/** State transition data structure. */
	struct xux
	{
		int u; ///< The action that has been executed.
		double * observed_state; ///< Beginning observation of the transition.
		double * next_observed_state; ///< Observation after executing the action.
		double reward; ///< Reward of the transition.
		bool is_terminal; ///< true if the transition ended in terminal state
		double Q; ///< Q value
	};

	void train(); ///< Start learning.
	
	/** Return best Q value and action given an observation. */
	double min_Q(const double * observation, double * action);

	/** Dimension of observation space. (equal to state space) */
	int observation_dim;

	/** Dimension of action space. */
	int action_dim;
  

	/** The action set */
	Aset aset;

	/** Reads options.
	 *  \param fname filename
	 *  \param chapter title of the section in the config
	 *  \return for success */
	bool read_options(const char *fname, const char *chapter);
	
	std::vector<ExtraTree *> model; ///< The tree ensemble.
	std::vector<xux> data;          ///< The training data.
	
	double epsilon;  ///< Parameter for greedy exploration.
	double gamma;    ///< Discount factor.
	
	int fq_cycles;   ///< Number of fitted Q iterations.
	int update_freq; ///< Frequency of learning.
	int episode;     ///< Count episodes.
	int num_trees;   ///< Number of trees in an ensemble.
	int num_tests;   ///< Number of dimensions to check when splitting.
  int num_threads; ///< Number of threads.
	int n_min;       ///< Minimum number of samples in a node.
	
	char fileprefix[255]; ///< Prefix of the filename used to load/save the ensemble.
	
};

#endif /* __TREECONTROLLER_H__ */
