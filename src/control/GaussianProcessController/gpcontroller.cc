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

#include "gpcontroller.h"
#include "rprop.h"
#include "valueparser.h"

#include <cmath>
#include <sstream>
#include <cstdio>

bool GaussianProcessController::init(const int observation_dim, const int action_dim, double deltat, const char* fname, const char* chapter)
{
	if (action_dim != 1) {
		EOUT("Error: This controller only supports discrete actions with one dimension.");
		return false;
	}
	this->observation_dim = observation_dim;
	update_freq = 1;
	epsilon = 0.5;
	gamma = 0.95;
	
	if (!this->read_options(fname, chapter)) {
		EOUT("Error on reading options.");
		return false;
	}
	if (update_freq > 0) {
		// init gp models, one for each action
    double p[100];
    int i = 0;
		stringstream ss (stringstream::in | stringstream::out);
		// read hyperparameters
    ss << hyperparam;
		while (ss.good()) { 
		  ss >> p[i];
      p[i] = log(p[i]);
      i++;
    }
	  for(int i = 0; i < aset.size(); ++i) {
			model.push_back(new libgp::GaussianProcess(observation_dim, covfunc));
			model.back()->covf().set_loghyper(p);
		}
	} else {
		char filename[255];
		for(int i = 0; i < aset.size(); ++i) {
			sprintf (filename, "%s%04i", fileprefix, i);
			model.push_back(new libgp::GaussianProcess(filename));
		}
	}
	sequence = 0;	
	return true;
}

void GaussianProcessController::deinit()
{
	while(!model.empty()) {
		delete model.back();
		model.pop_back();
	}
	while(!data.empty()) {
		delete [] data.back().observed_state;
		delete [] data.back().next_observed_state;
		data.pop_back();
	}
}

bool GaussianProcessController::get_action(const double* observed_state, double* action)
{
	// epsilon-greedy with learned controller, random action otherwise
	if (drand48() >= epsilon && sequence >= update_freq) {
		min_Q(observed_state, action);
	} else {
		int action_def = drand48() * aset.size();
		aset.getAction(action_def, action, 1);
	}
	return true;
}

void GaussianProcessController::notify_transition(const double* observed_state,
		               const double* action,
		               const double* next_observed_state,
		               const double reward, 
		               const bool is_terminal_state, 
		               const double terminal_reward)
{
	if (update_freq == 0) return;
	xux pattern;
	pattern.observed_state = new double[observation_dim];
	pattern.next_observed_state = new double[observation_dim];
	for (int i=0; i<observation_dim; ++i) {
		pattern.observed_state[i] = observed_state[i];
		pattern.next_observed_state[i] = next_observed_state[i];
	}
	int i;
	double a[1];
	for(i = 0; i < aset.size(); ++i) {
		aset.getAction(i, a, 1);
		if (fabs(a[0] - action[0]) < 0.001) break;
	}
	pattern.u = i;
	pattern.is_terminal = is_terminal_state;	
	pattern.reward = reward+terminal_reward;
	data.push_back(pattern);
}

void GaussianProcessController::notify_episode_stops(const double* current_observed_state)
{
	++sequence;
	if (update_freq != 0 && sequence%update_freq == 0) train();
}

bool GaussianProcessController::read_options(const char * fname, const char * chapter)
{
	if(fname == 0) return true;
	char param[512];	
	ValueParser vp(fname, chapter==NULL?"Controller":chapter);
	vp.get("epsilon",epsilon);
	vp.get("fq_cycles",fq_cycles);
	vp.get("gamma",gamma);
	vp.get("activeset_size",activeset_size);
	vp.get("update_freq",update_freq);
	vp.get("actions", param, MAX_STR_LEN);
 	aset.parseStr(param, 1);
	vp.get("covfunc", param, MAX_STR_LEN);
	covfunc = string(param);
	vp.get("fileprefix",fileprefix,MAX_STR_LEN);
	vp.get("hyperparam", hyperparam, MAX_STR_LEN);
	return true;
}

void GaussianProcessController::train()
{
  double action[1];
  IOUT("Start training using " << data.size() << " samples ...");
  
  int counter[model.size()];
  for(int j = 0; j < fq_cycles; ++j) {
    int n = 0;
    // for all training samples
    for(size_t i = 0; i < data.size(); i++) {
      // immediate costs			
      data[i].Q = data[i].reward;
      if (!data[i].is_terminal) data[i].Q += gamma*min_Q(data[i].next_observed_state, action);
    }
    // reset counter
    for (size_t i=0; i<model.size(); ++i) {
      counter[i] = 0;
      n += model[i]->get_sampleset_size();
    }
    // update old target values
    for(size_t i = 0; i < (size_t)n; ++i) {
      model[data[i].u]->set_y(counter[data[i].u]++, data[i].Q);
    }
    // add new input/target pairs
    for(size_t i = n; i < data.size(); i++) {
      model[data[i].u]->add_pattern(data[i].observed_state, data[i].Q);
    }
  }
#if 0
  for(size_t i = 0; i < model.size(); ++i) {
    libgp::RProp rprop;
    rprop.maximize(model[i]);
    std::cout << model[i]->covf().get_loghyper().transpose().array().exp() << std::endl;
#endif
  
  char filename[255];
  for(int i = 0; i < aset.size(); ++i) {
    sprintf(filename, "%s%04i", fileprefix, i);
    model[i]->write(filename);
  }
}

double GaussianProcessController::min_Q(const double * observation, double * action)
{
	double mn = -log(0);
	int action_def = 0;
	for (size_t j=0; j<model.size(); j++) {
		double q = model[j]->f(observation);
		if (q < mn) {
			mn = q;
			action_def = j;
		}
	}
	aset.getAction(action_def, action, 1);
	return mn;
}

REGISTER_CONTROLLER( GaussianProcessController , "Fitted Q with full Gaussian processes.");
