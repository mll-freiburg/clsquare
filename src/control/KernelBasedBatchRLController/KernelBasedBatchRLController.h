/*
clsquare - closed loop simulation system
Copyright (c) 2010, 2011 Machine Learning Lab, 
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

#ifndef _KERNEL_BASED_RL_CONTROLLER_
#define _KERNEL_BASED_RL_CONTROLLER_

#include <string>
#include "AbstractBatchController.h"
#include <vector>
#include <fstream>
#include <queue>
#include <set>
#include <ANN/ANN.h>


namespace KBRL {
  
  using BatchUtils::AbstractBatchController;
  using BatchUtils::XUiX;
  using namespace std;
  
  typedef double* ValueArray; ///< This type will be used to store the q-target-values of the stored transitions.

  /** The KQ-class is a (K)ernel-based representation of the (Q)-function. 
   The class stores q-target values for a given set of state-action pairs.
   When it's asked to return the Q-value of some new state-action pair,
   it uses weighted averaging to calculated the approximated q-value from
   the stored samples. When calculating this average, this approximator
   uses only the N-closest samples (thus, only a finite number and not all
   samples) and either a gaussian or euclidian kernel to weight the influence
   of individual samples according to their distance to the querry. It's
   important to note that this approximator works with discrete actions only
   and uses only the subset of samples with the same action as the querry
   in order to calculate an approximation. Furthermore, the weights of 
   samples are guarranted to be positive and summing up to one, thus 
   fulfilling the conditions for being an "averager" as defined by Gordon,
   ICML, 1995. 
   
   To speed up the search for the n-nearest neighbors when asking the
   approximatior for a q-value, this implementation uses a kd-tree for 
   accessing the samples. @author Sascha Lange */
  class KQ {
  public:
    int d;                         ///< the dimension of the sate space (points in the kd-tree)
    unsigned int num_actions;      ///< the number of discrete actions. 
    int k;                         ///< the number of samples to use in the k-nearest-neighbors approximation.

    vector<ANNpointArray> points;  ///< these are the samples (state-part of state-action pairs) that are used by this approximator. There is an individual array for each action.
    vector<int> numPoints;         ///< this array stores the number of samples of every action
   
    /** The default constructor of the Kernel-Based-Q-Approximator. 
     Immediately constructs a kd-tree on basis of the given XUiX-data. 
     \param xuix a list of transitions from state to next state when using
                 action ui, where ui just specifies the index of the 
                 discrete aciton.
     \param d the dimension of the state space.
     \param num_actions the number of actions, where num_actions-1 must be the
                        highest index of the actions 
     \param qinit the initial Q-value 
     \param k the number of samples to use in k-nearest-neighbour averaging
     \param useAutoScaleGaussian whether or not to automatically "scale" the
                                 size of the averaging kernel with the density
                                 of the available data in the surrounding of
                                 the querry. */
    KQ(vector<XUiX>& xuix, int d, unsigned int num_actions, double qinit=0., 
       int k=10, bool useAutoScaleGaussian=false);
    KQ(vector<XUiX>& xuix, int d, unsigned int num_actions, const KQ& initFrom, 
       int k=10, bool useAutoScaleGaussian=false);

    ~KQ(); ///< Destructor of the KQ-Approximator. Cleans up the kd-tree.
      
    /** This constructor reads a kernel-based Q-Approximation from a file. */
    KQ(const std::string& filename);
    
    /** returns the index of the action with the highest q-value for the
     given state */
    int getOptimalAction(const ANNpoint point) const;
    /** returns the q-value of the given state-action pair */
    double getQValue(const ANNpoint point, int ui) const;
    /** returns the state-value of the given state. */
    double getStateValue(const ANNpoint point) const;
    
    /** return the q-value of the given state-action pair, where the point
     is specified by its index in the points-array. Thus, this method only
     works for points stored in the approximator but vastly speeds up the
     access to the nearest neighbors (internally, the object uses a 
     pre-calculated list of pointers to the k-nearest-neighbors of each 
     stored point). This method is used by the controller during the batch
     learning procedure. */
    double getQValue(int pidx, int ui) const;
    /** returns the state-value of the given state, where the state
     is specified by its indes in the points-array. See the corresponding
     method getQVlaue(int pidx, int ui) for further details. */
    double getStateValue(int pidx) const;
    
    /** returns a reference to the array that holds the present q-values 
     of the stored state-action pairs. 
     \attention The vales in the returned array must not be changed; 
     use updtValues() instead! */
    const vector<ValueArray>& values() const { return *vp; }    
    /** returns a reference to an array that can be used to store 
     intermediate results during updating the q-values of the stored states */
    vector<ValueArray>& updtValues() { return *vpu; }        
    /** switches the arrays holding the present and updated Q-values. Should
     be called after updates for all Q-values have been calculated and stored
     in the updtValues() - array. */
    void switchValues() { vector<ValueArray>* tmp = vp; vp = vpu; vpu = tmp; }
    
    friend std::ostream& operator<<(ostream& out, const KQ& kq);
    
  protected:
    vector<ANNkd_tree*> kdtrees;  ///< array of num_actions KD-Trees; one individual tree for the samples of each action
    vector<ValueArray> values1;   ///< array to hold q-values (either the present or updated q-values)
    vector<ValueArray> values2;   ///< array to hold q-values (either the present or updated q-values)
    vector<int*> nnidx;           ///< internal references to the nearest-neighbors of all stored points
    vector<double*> dists;        ///< distances between points

    vector<ValueArray>* vp;       ///< pointer to the array holding the present q-values
    vector<ValueArray>* vpu;      ///< pointer to the array holding the updated q-values
    
    bool ownPoints;               ///< whether or not the approximator owns the samples. Only if the approximator was read from disk, it owns the samples and will free the corresponding memory in the destructor.
    bool useAutoScaleGaussian;    ///< flag to indicate whether or not to use the auto-scaling of the kernels
    
    void buildIndex();            ///< this method actually builds the kd-trees
  };
  std::ostream& operator<<(ostream& out, const KQ& kq);

};

 
/** Implements a batch RL controller that uses lazy learning with a 
 kernel-based approximation of the Q-function. This controller is a variant
 of the algorithm proposed by Ormoneit and Sen, Machine Learnig, 2002. 
 @author Sascha Lange. 
 @ingroup CONTROLLER
 @ingroup LEARNING
 */
class KernelBasedRLController  : public BatchUtils::AbstractBatchController {
public:
  
  enum { APPROX_KADP };   ///< presently not used, but may integrate alternative representations later on
  
  KernelBasedRLController();
  virtual ~KernelBasedRLController();
  
  /** Computes an action, given an observation of a state.
   * \param observation observation of current state
   * \param action action to execute in current state.
   * \return true, for success. */ 
  virtual bool get_action(const double* observed_state, double* action);
        
 protected: 
  
  double epsilon;
  double qinit;
  double gamma;
  int k;
  
  int stepsVPlotsResult;   ///< How often should the resulting vplots of the batch training procedure be written to disk. 0 = never.
  int stepsVPlotsLearning; ///< How often should the the programm write all intermediate vplots within a single batch training procedure to disk. 0 = never.
  
  int threads;
    
  bool useAutoScaleGaussian;
  
  /** method that updates the q-function given the present data */
  virtual void do_learning();
  
  /** Reads options. 
   *  \param fname filename 
   * \return for success */
  virtual bool read_options(const char *fname, const char *chapter);
  
  void writeVPlot(const std::string& filename);
    
  bool policyAvailable;
  KBRL::KQ* kq;
  
  const void getEpsilonGreedyAction(const double* x, double* action, double epsilon) const;
  const void getGreedyAction(const double* x, double* action) const;
  const int  getGreedyActionIndex(const double* x) const;
  
  
};




#endif
