/*
 clsquare - closed loop simulation system
 Copyright (c) 2011 Machine Learning Lab, 
 Prof. Dr. Martin Riedmiller, University of Freiburg
 
 Author: Sascha Lange
 
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

#ifndef _ABSTRACT_BATCH_CONTROLLER_
#define _ABSTRACT_BATCH_CONTROLLER_

#include "controller.h"
#include <vector>
#include <string>
#include <fstream>
#include "BatchData.h"


namespace BatchUtils {
  
  using std::vector;
  using std::string;
    
  /** Implements a batch RL controller that uses lazy learning with a 
   kernel-based approximation of the Q-function. This controller is a variant
   of the algorithm proposed by Ormoneit and Sen, Machine Learnig, 2002. 
   @author Sascha Lange.
   @ingroup CONTROLLER
   @ingroup LEARNING
   */
   
  class AbstractBatchController  : public Controller {
  public:
    
    AbstractBatchController();
    virtual ~AbstractBatchController();
    
    /** Notifies the controller of the last transition that did occur. This
     batch controller simply collects all transitions and stores them for
     later use in the batch training procedure. */
    virtual void notify_transition(const double* observed_state,
                                   const double* action,
                                   const double* next_observed_state,
                                   const double reward, 
                                   const bool is_terminal_state, 
                                   const double terminal_reward);
    
    /** Notifies the controller that the episode has been stopped. This 
     notification will start the learning process after every episode. */
    virtual void notify_episode_stops(const double* current_observed_state);
    
    
    /** Initialize controller
     * \param observation_dim dimension of observation space
     * \param action_dim dimension of action space. 
     * \param deltat duration of one control cycle [s]
     * \param fname file, which contains configuration of controller
     * \param chapter chapter in which the configuration is located
     * \return true for success. */
    virtual bool init(const int observation_dim, const int action_dim, double deltat, const char* fname=0, const char* chapter=0);
    
    /** Called from mainloop during deconstruction of CLSquare. */
    virtual void deinit(); 
    
  protected: 
    
    bool learn;              ///< Should learn or not?
    bool verbose;            ///< Should log warnings and info messages?
    
    int observation_dim;     ///< Dimension  of observation space. 
    int action_dim;          ///< Dimension  of action space.
    double* action_def;      ///< Definition of actions. 
    
    unsigned int num_actions;
    int* actions_in_dim;     ///< Counts actions in every dimensions.
    vector<vector<double> > all_actions;
    
    int episodeCount; 
    
    std::vector<XUX> xux;  
    
    /** method that updates the q-function given the present data */
    virtual void do_learning() =0;
    
    /** Reads options. 
     *  \param fname filename 
     * \return for success */
    virtual bool read_options(const char *fname, const char *chapter);
    
    /** Parses defintion of actions from string 
     * \param param action string */   
    virtual void parse_actions(const char* param);
    
    /** helper for parsing actions. */
    int av2ai(const int* action_value);
    /** helper for parsing actions. */
    void av2a(const int* action_value, double* action);
    /** helper for parsing actions. */
    void ai2av(int ai, int* actionValue);
    
    void writeXUX(const string& filename);
    void readXUX(const string& filename);
    
  };
  
}




#endif
