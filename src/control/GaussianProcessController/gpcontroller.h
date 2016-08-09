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

#ifndef __GPCONTROLLER_H__
#define __GPCONTROLLER_H__

#include "controller.h"
#include "aset.h"
#include "setdef.h"
#include "gp.h"

#include <vector>
#include <string>

/** Uses Gaussian Processes to learn a policy.
  * Based on libgp; for more detailed information, see 
  * https://bitbucket.org/mblum/libgp/wiki
  *
  * Available parameters are:
  * \li \b update_freq: update frequency (0 for testing, >=1 for training)
  * \li \b gamma: discount rate
  * \li \b epsilon: exploration rate (0 for testing)
  * \li \b fq_cycles: cycles for fitted Q iteration
  * \li \b fileprefix: prefix for model files
  * \li \b covfunc: covariance function; see libgp documentation for details
  * \li \b hyperparam: covariance function hyper-parameters; see libgp documentation for details
  *
  * @ingroup CONTROLLER
  * @ingroup LEARNING */
class GaussianProcessController : public Controller
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
	 \return false, if the controller wants to stop the present sequence. */
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

	/** called by the kernel whenever a sequence has ended. */
	void notify_episode_stops(const double* current_observed_state);


private:
	struct xux
	{
		int u;
		double * observed_state;		
		double * next_observed_state;
		double reward;
		bool is_terminal;
		bool active;
		double Q;
	};
	
	void train();
	
	double min_Q(const double * observation, double * action);

	/** Dimension of observation space. (equal to state space) */
	int observation_dim;

	/** Action set */
	Aset aset;

	/** Reads options.
	 *  \param fname filename
	 *  \return for success */
	bool read_options(const char *fname, const char *chapter);

	std::string covfunc;
	
	std::vector<libgp::GaussianProcess *> model;
	std::vector<xux> data;
	
	double epsilon;
	double gamma; 
	
	int fq_cycles;
	int update_freq;
	int sequence;
	int activeset_size;
	
	char fileprefix[255];
  char hyperparam[255];
	
};

#endif /* __GPCONTROLLER_H__ */
