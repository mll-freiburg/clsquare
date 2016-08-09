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

#include "treecontroller.h"
#include "valueparser.h"
#include <cmath>
#include <cstdlib>
#include <cassert>
#include <cstring>
#include <limits>

bool TreeController::init(const int observation_dim, const int action_dim, double deltat, const char* fname, const char* chapter)
{
	this->observation_dim = observation_dim;
	this->action_dim = action_dim;
	
	if (!this->read_options(fname,chapter)) {
		EOUT("Error on reading options.");
		return false;
	}
	if (update_freq != 0) {
		// init tree models, one for each action
		for(int i = 0; i < aset.size(); ++i) {
			model.push_back(new ExtraTree(observation_dim));
		}
	} else {
		char filename[255];
		for(int i = 0; i < aset.size(); ++i) {
			sprintf (filename, "%s%03i", fileprefix, i);
			model.push_back(new ExtraTree(filename));
		}
	}
	episode = 0;	
	return true;
}

void TreeController::deinit()
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

bool TreeController::get_action(const double* observed_state, double* action)
{
	// epsilon-greedy with learned controller, random action otherwise
	if (drand48() >= epsilon && episode >= update_freq) {
		min_Q(observed_state, action);
	} else {
		int action_def = drand48() * aset.size();
		aset.getAction(action_def, action, action_dim);
	}
	return true;
}

void TreeController::notify_transition(const double* observed_state,
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
  memcpy(pattern.observed_state, observed_state, sizeof(double)*observation_dim);
  memcpy(pattern.next_observed_state, next_observed_state, sizeof(double)*observation_dim);
	int i, j;
	double a[action_dim];
	for(i = 0; i < aset.size(); ++i) {
    aset.getAction(i, a, action_dim);
    for (j=0; j<action_dim; ++j) {
      if (fabs(a[j] - action[j]) > std::numeric_limits<double>::epsilon()) break;
    }
    if (j==action_dim) break;
	}
  assert(i!=aset.size());
	pattern.u = i;
	pattern.is_terminal = is_terminal_state;	
	pattern.reward = reward;
	if (is_terminal_state) pattern.reward += terminal_reward;
	data.push_back(pattern);
}

void TreeController::notify_episode_stops(const double* current_observed_state)
{
	++episode;
	if (update_freq != 0 && episode%update_freq == 0) train();
}

bool TreeController::read_options(const char * fname, const char * chapter)
{
	if(fname == 0) return true;
	char param[512];	
	ValueParser vp(fname, chapter==0 ? "Controller" : chapter);
	vp.get("epsilon", epsilon, 0.5);
	vp.get("fq_cycles", fq_cycles, 3);
	vp.get("gamma", gamma, 0.95);
	vp.get("num_trees", num_trees, 50);
	vp.get("num_tests", num_tests, observation_dim);
	vp.get("n_min", n_min, 10);
	vp.get("num_threads", num_threads, 4);
	vp.get("update_freq", update_freq, 1);
	vp.get("actions", param, 512);
 	aset.parseStr(param, 1);
	vp.get("fileprefix", fileprefix, 255);
  // use at least one thread
  num_threads = num_threads < 1 ? 1 : num_threads;
  // use at most as many threads as trees in the ensemble
  num_threads = num_threads > num_trees ? num_trees : num_threads;
	return true;
}

void TreeController::train()
{
	double action[action_dim];
	IOUT("Start training using " << data.size() << " samples ...");
	for(int j = 0; j < fq_cycles; ++j) {
		for(size_t i = 0; i < data.size(); i++) {
      xux *pattern = &data[i];
			pattern->Q = pattern->reward;			
			if (!pattern->is_terminal) pattern->Q += gamma*min_Q(pattern->next_observed_state, action);
      model[pattern->u]->add_pattern(pattern->observed_state, pattern->Q);	  
    }    
		for(size_t i = 0; i < model.size(); ++i) {
			model[i]->train(num_trees, num_tests, n_min, num_threads);
			model[i]->clear_sampleset();
		}
	}
	char filename[255];
	for(int i = 0; i < aset.size(); ++i) {
		sprintf(filename, "%s%03i", fileprefix, i);
		model[i]->write(filename);
	}
}

double TreeController::min_Q(const double * observation, double * action)
{
  vector<int> min_i;
  double min_v = INFINITY;
  double tmp;
  for (size_t i=0; i < model.size(); i++) {
    tmp = model[i]->predict(observation);
    if (tmp < min_v) {
      min_v = tmp;
      min_i.assign(1, i);
    } else if (tmp == min_v) {
      min_i.push_back(i);
    }
  }
  aset.getAction(min_i[lrand48() % min_i.size()], action, 1);
  return min_v;
}

REGISTER_CONTROLLER( TreeController , "Fitted Q Iteration using extra trees.");
