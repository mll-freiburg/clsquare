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
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN 
IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
*/

#include "AbstractBatchController.h"
#include "KernelBasedBatchRLController.h"
#include "valueparser.h"
#include "str2val.h"
#include <cstdlib>
#include <fstream>
#include "global.h"
#include <sstream>
#include "random.h"
#include "setdef.h"
#include <sys/stat.h>
#include <cassert>

using namespace BatchUtils;
using namespace std;
using namespace KBRL;

#define MAX_NUM_ACTIONS 255

#define VERBOSE(xxx) if(verbose) cout <<__PRETTY_FUNCTION__ <<": "<< xxx<<endl


// Variants for MINIMIZING costs
//#define OPT_FIRST_ARG_BETTER(x,y) ((x)<(y))
//#define OPT_FIRST_ARG_BETTER_OR_EQUAL(x,y) ((x)<=(y))

// Variants for MAXIMIZING rewards
#define OPT_FIRST_ARG_BETTER(x,y) ((x)>(y))
#define OPT_FIRST_ARG_BETTER_OR_EQUAL(x,y) ((x)>=(y))


#ifdef XCode
#pragma mark -
#pragma mark Object Life Cycle
#endif 


KernelBasedRLController::KernelBasedRLController()
 : epsilon(.3), qinit(0.), gamma(1.), k(10), stepsVPlotsResult(0), stepsVPlotsLearning(0), 
   threads(1), policyAvailable(false), kq(0)
{}

KernelBasedRLController::~KernelBasedRLController()
{}


 



#ifdef XCode
#pragma mark -
#pragma mark Kernel-Based Batch Reinforcement Learning
#endif


class WorkerDataKADP {
public:
  pthread_t threadId;
  
  int start;
  int step;
  
  vector<XUiX>* xuix;
  KQ* kq;
  
  int num_actions;
  double gamma;
  
  double maxChange;
  
  WorkerDataKADP(int start, int step, vector<XUiX>* xuix, KQ* kq, int num_actions, double gamma) 
  : start(start), step(step), xuix(xuix), kq(kq), num_actions(num_actions), gamma(gamma), maxChange(0)
  {}
  
  WorkerDataKADP() {}
};


/* the following method is called by each thread with the thread's individual set
 of parameters stored in arg. The method calculates a dp-update along a part of the
 collected transition samples and stores new target values for the corresponding 
 state-action pair. */
static void* parallel_kadp(void* arg) 
{
  WorkerDataKADP* data = static_cast<WorkerDataKADP*>(arg); // arguments of the individual thread
  
  vector<int> p(data->num_actions, 0);                  // this array holds "pointers" to the next entry to process in the kq.points-vector for each individual action
  for (unsigned int i=0; i < data->xuix->size(); i++) { // although we only update a fraction of the stored transitions within a single thread we have to walk through the whole list in order to always have the right values in the array p.
    int ui = (*data->xuix)[i].ui;                       // this is the index of the action used in the "present" transition
    
    // THIS IS A BUG: if ((i-data->start) % data->step == 0) { 
    // changed to:
    if ( (int)(i % data->step) == data->start) {               // true, if this thread should process this particular transition and false in case it'll be processed by another thread
      
      if ((*data->xuix)[i].isTerminal) {
        data->kq->updtValues()[ui][p[ui]] = (*data->xuix)[i].r;       // DP Update for terminal states.
      }
      else {
        data->kq->updtValues()[ui][p[ui]] = (*data->xuix)[i].r + data->gamma * data->kq->getStateValue(i); // DP Update for non-terminal states.
      } // ATTENTION: this relies on the property of kq->getStateValue(i) to return the state value of the ENDING STATE of transition i 
      
      if (fabs(data->kq->updtValues()[ui][p[ui]] - data->kq->values()[ui][p[ui]]) > data->maxChange) {     // check for delta
        data->maxChange = fabs(data->kq->updtValues()[ui][p[ui]] - data->kq->values()[ui][p[ui]]);
      }
    }
    
    p[ui]++;  // this update of the present-sample "pointer" is the reason we have to also walk through those transitions we don't process in this thread
  } 
  
  return 0;
}



void KernelBasedRLController::do_learning ()
{
  cerr << "Start learning with " << xux.size() << " samples." << endl;

  if (xux.size() < 10) {
    cerr << "Less than 10 samples, skip learning phase." << endl; 
    return ;
  }

  double maxChange, threshold = 0.01;    // a threshold on the bellman error for determining when to stop dynamic processing
  vector<XUiX> xuix;
  translate(xux, xuix, all_actions);     // convert the xux representation to transitions with indicees of the action
 
  if (kq) {                   
    delete kq; kq = 0;
  }
  kq = new KQ(xuix, observation_dim, num_actions, qinit, k); // create a new, qinit-initialized approximation

  int numThreads = threads < 1 ? 1 : threads;      // use at least one thread
  vector<WorkerDataKADP> workerData(numThreads);   // vector will hold the individual parameters of each thread
  
  int numIterations=0;
  do { 
    maxChange = 0.;
    
    for (int thread=0; thread < numThreads; thread++) {   // start the threads with the appropriate set of parameters indicating which part of the data to process
      workerData[thread] = WorkerDataKADP(thread, numThreads, &xuix, kq, num_actions, gamma);
      if (thread == numThreads-1) parallel_kadp(&workerData[thread]);
      else pthread_create(&(workerData[thread].threadId), 0, parallel_kadp, (void*) &workerData[thread]);
    }
    for (int thread=0; thread < numThreads-1; thread++) { // now wait for all threads to finish their work
      pthread_join(workerData[thread].threadId, 0);
      if (maxChange < workerData[thread].maxChange) maxChange = workerData[thread].maxChange;
    }
    if (maxChange < workerData[numThreads-1].maxChange) maxChange = workerData[numThreads-1].maxChange;
    
    kq->switchValues();  // switch value arrays; updtValues() become present values() used by the K-nearest-neighbours q-approximator
    
    numIterations++;     
    
    if (stepsVPlotsLearning > 0 && episodeCount % stepsVPlotsLearning == 0) { // should write extensive data of the learning progress to disk?
      char pathname[500];
      char filename[500];
      
      sprintf(pathname, "episode_%.4d", episodeCount);
      mkdir(pathname, 0777);
      
      sprintf(filename, "%s/vplot_%.4d.data", pathname, numIterations);
      writeVPlot(filename);        
    }
    
  } while (maxChange > threshold); // stop as soon as the "bellman error" (maximal change under max-norm) becomes smaller than the given threshold
  
  policyAvailable = true;  // let the controller know it now has a policy available (greedy exploitation of the q-approximator)
  
  cerr << "DP needed " << numIterations << " iterations to converge training an approximator with " << xux.size() << " instances." << endl;
  char filename[500];
  sprintf(filename, "kq_%.4d.approx", episodeCount);
  ofstream out(filename);
  if (out) {
    out << *kq << endl;  // write the KQ-approximation to disk
    out.close();
  }
  ofstream out2("tmp.approx");
  if (out2) {
    out2 << *kq << endl; // this temporary file will be read by the controller started for testing the policy
    out2.close();
  }
  
  if (stepsVPlotsResult > 0 && episodeCount % stepsVPlotsResult == 0) { // should write data that is used to generate a plot of the state-value-function?
    sprintf(filename, "vplot_%.4d.data", episodeCount);
    writeVPlot(filename);
  }
  
  if (episodeCount % 10 == 0) {   // write all transition - samples to disk?
    sprintf(filename, "samples_%.4d.xux", episodeCount);
    writeXUX(filename);
  }
  
}



#ifdef XCode
#pragma mark -
#pragma mark Interface to CLSquare
#endif


bool KernelBasedRLController::get_action(const double* observed_state, double* action)
{  
  if (! policyAvailable) {  // random action (no controller available)
    getEpsilonGreedyAction(observed_state, action, 1.);
  }
  else {
    if (learn) {  // explore?  
      getEpsilonGreedyAction(observed_state, action, epsilon);
    }
    else {        // exploit!
      getGreedyAction(observed_state, action);
    }
  }
  return true;
}

#ifdef XCode
#pragma mark -
#pragma mark Selecting Actions
#endif 

const void KernelBasedRLController::getEpsilonGreedyAction(const double* x, double* action, double epsilon) const
{
  if (Util::brandom(epsilon)) {
    int a = lrand48() % num_actions;  // choose a random action
    for (unsigned int i=0; i < all_actions[a].size(); i++) { // and copy its values to the action-vector
      action[i] = all_actions[a][i];
    }   
  }
  else {
    return getGreedyAction(x,action); // choose a greedy action 
  }
}

const void KernelBasedRLController::getGreedyAction(const double* x, double* action) const
{
  int a = getGreedyActionIndex(x);
  for (unsigned int i=0; i < all_actions[a].size(); i++) {
    action[i] = all_actions[a][i];
  }
}


// finds the index of the best action for the given state 
// according to the present approximation of the Q-function
const int KernelBasedRLController::getGreedyActionIndex(const double* x) const
{
  vector<int> maxi;   
  vector<double> maxv;
  maxi.push_back(0);
  maxv.push_back(kq->getQValue((const ANNpoint)x, 0));
  double tmp;
  for (unsigned int i=1; i < all_actions.size(); i++) {
    tmp = kq->getQValue((const ANNpoint)x, i);
    if (OPT_FIRST_ARG_BETTER_OR_EQUAL(tmp,  maxv[0])) {   // present entry is better or equally good as the best value so far
      if (OPT_FIRST_ARG_BETTER(tmp, maxv[0])) {           // present entry is the single biggest so far
        maxi.clear();
        maxv.clear();
      }
      maxi.push_back(i);
      maxv.push_back(tmp);
    }
  }
  return maxi[lrand48() % maxi.size()];  // if there's more than one best action to choose from, select one randomly 
}

#ifdef XCode
#pragma mark -
#pragma mark Reading and Parsing Options
#endif 

bool KernelBasedRLController::read_options(const char * fname, const char * chapter) {
  if (! AbstractBatchController::read_options(fname,chapter)) return false; // read parameters of super class
      
  ValueParser vp(fname,chapter==NULL?"Controller":chapter);
  char param[255];
  
  vp.get("epsilon",epsilon,.3);
  vp.get("gamma",gamma,1.);
  vp.get("threads",threads, 1);
  vp.get("qinit",qinit, 0.);
  vp.get("k", k, 10);
  vp.get("use_autoscale_gaussian", useAutoScaleGaussian, false);
  
  vp.get("vplot_outer_steps", stepsVPlotsResult, 10);
  vp.get("vplot_inner_steps", stepsVPlotsLearning, 0);
  
  if (! learn) {
    vp.get("load_kq", param, 255);
    kq = new KQ(param);
    
    ofstream out("cat.approx");
    if (out) {
      out << *kq << endl;
      out.close();
    }
    
    policyAvailable = true;
  }
  
  return true;
}


#ifdef XCode
#pragma mark -
#pragma mark Input / Output
#endif 


void KernelBasedRLController::writeVPlot(const string& filename)
{
  if (!policyAvailable || !kq) {
    return ;
  }
  
  if (observation_dim != 2) {  // presently, this works only for the mountain car 
    return ;
  }
  
  ofstream out(filename.c_str());
  if (!out) {
    return ;
  }
  
  double xmin = -1.4;
  double xmax =  1.0;
  double ymin = -4.;
  double ymax =  3.5;
  
  int numCells = 100;
  
  double vmax = 0.;
  double vmin = -50.;
  
  
  double xStep = (xmax - xmin) / numCells;
  double yStep = (ymax - ymin) / numCells;
  
  for (double y =ymin + yStep/2.; y <= ymax; y+=yStep) {    // wirte a gnu-plotable grid of state-values
    for (double x = xmin + xStep/2.; x <= xmax; x+=xStep) {
      double point[2] = {x, y};
      double V = kq->getStateValue(&point[0]);
      V = V > vmax ? vmax : V < vmin ? vmin : V;
      out << x << " " << y << " " << V << endl;
    }
    out << endl;
  }
  out.close();
}



#ifdef XCode
#pragma mark -
#pragma mark Kernel-Based Approximation of Q-Function
#endif

KQ::KQ(const std::string& filename) {
  ifstream in(filename.c_str()); 
  if (!in) {
    cerr << "Could not open " << filename << " for reading." << endl;
    exit(1);
  }
  
  // Read from start of KQ-file:
  in >> num_actions >> d >> k; // NUMBER_OF_ACTIONS DIMENSION_OF_STATE_SPACE NUMBER_OF_NEAREST_NEIGHBOURS_TO_USE
  
  points.resize(num_actions);
  numPoints.resize(num_actions);
  values1.resize(num_actions);  
  kdtrees.resize(num_actions);
  
  for (unsigned int i=0; i < num_actions; i++) {
    in >> numPoints[i];        // for each possible action the file now contains a number specifying the number of samples for this particular action
  }
  
  for (unsigned int i=0; i < num_actions; i++) { // repeat for each action
    
    points[i] = (ANNpointArray) new ANNpoint[numPoints[i]];  //(ANNpointArray) calloc(xuix.size() , sizeof(ANNpoint));
    values1[i] = (ValueArray) new double[numPoints[i]];       //(ValueArray) calloc(xuix.size(), sizeof(double));
    
    for (int p =0; p < numPoints[i]; p++) {     // read all points for the present action
      points[i][p] = new ANNcoord[d];
      assert (in); 
      for (int j=0; j < d; j++) {
        in >> points[i][p][j];
      }
      in >> values1[i][p];
    }
    
    kdtrees[i] = new ANNkd_tree(points[i], numPoints[i], d); // and create a kd-tree for each action indexing its samples
  }
  in.close();
  ownPoints = true;
  vp = &values1;
}


KQ::KQ(vector<XUiX>& xuix, int d, unsigned int num_actions, double qinit, int k, bool useAutoScaleGaussian) 
: d(d), num_actions(num_actions), k(k), useAutoScaleGaussian(useAutoScaleGaussian)
{
  points.resize(num_actions);
  numPoints.resize(num_actions);
  values1.resize(num_actions);  
  values2.resize(num_actions);  
  
  for (unsigned int i=0; i < num_actions; i++) {
    points[i] = (ANNpointArray) new ANNpoint[xuix.size()];  //(ANNpointArray) calloc(xuix.size() , sizeof(ANNpoint));
    values1[i] = (ValueArray) new double[xuix.size()];       //(ValueArray) calloc(xuix.size(), sizeof(double));
    values2[i] = (ValueArray) new double[xuix.size()];       //(ValueArray) calloc(xuix.size(), sizeof(double));
    numPoints[i] = 0;
  }
  
  vp = &values1;
  vpu = &values2;
 

 
  for (unsigned int i=0; i < xuix.size(); i++) {
    ANNpoint point = new ANNcoord[d];
    for (int j=0; j < d; j++) {
      point[j] = xuix[i].x[j];
    }
    points[xuix[i].ui][numPoints[xuix[i].ui]] = point;
    (*vp)[xuix[i].ui][numPoints[xuix[i].ui]] = qinit;
    numPoints[xuix[i].ui]++;
  }
  
  kdtrees.resize(num_actions);
  for (unsigned int i=0; i < num_actions; i++) {
    //cerr << "Creating kd-tree #" << i << " with " << numPoints[i] << " points." << endl;
    kdtrees[i] = new ANNkd_tree(points[i], numPoints[i], d);

    //cerr << "Num Points in kdtree " << i << ": " << kdtrees[i]->nPoints() << endl;    
    //cerr << "dim: " << kdtrees[i]->theDim() << endl;

    //ofstream out("tree.log", fstream::app);
    //kdtrees[i]->Print(ANNtrue, out);
    //out.close();
  }
 
  
  { // construct index arrays for next-states in xuix
    nnidx.resize(num_actions);
    dists.resize(num_actions);
    
    for (unsigned int a=0; a < num_actions; a++) {
      int n = numPoints[a];
      if (k < n) n = k;  /// k-nearest neighbour, but don't lookup more neighbours than elements.
      
      nnidx[a] = new int[xuix.size() * n];
      dists[a] = new double[xuix.size() * n];
      
      for (unsigned int p=0; p < xuix.size(); p++) {
        
kdtrees[a]->annkSearch(&xuix[p].xnext[0], n, &nnidx[a][n*p], &dists[a][n*p], 0.0); // last arg: precision (0=100%, 0.1 = up too 10% farther away)
      }
    }
  }
  ownPoints = false;
}



KQ::KQ(vector<XUiX>& xuix, int d, unsigned int num_actions, const KQ& initFrom, int k, bool useAutoScaleGaussian) 
: d(d), num_actions(num_actions), k(k), useAutoScaleGaussian(useAutoScaleGaussian)
{

  cerr << "Initializing with old kq." << endl;
  
  points.resize(num_actions);
  numPoints.resize(num_actions);
  values1.resize(num_actions);  
  values2.resize(num_actions);  
  
  for (unsigned int i=0; i < num_actions; i++) {
    points[i] = (ANNpointArray) new ANNpoint[xuix.size()];  //(ANNpointArray) calloc(xuix.size() , sizeof(ANNpoint));
    values1[i] = (ValueArray) new double[xuix.size()];       //(ValueArray) calloc(xuix.size(), sizeof(double));
    values2[i] = (ValueArray) new double[xuix.size()];       //(ValueArray) calloc(xuix.size(), sizeof(double));
    numPoints[i] = 0;
  }
  
  vp = &values1;
  vpu = &values2;
  
  for (unsigned int i=0; i < xuix.size(); i++) {
    points[xuix[i].ui][numPoints[xuix[i].ui]] = &xuix[i].x[0];
    (*vp)[xuix[i].ui][numPoints[xuix[i].ui]] = initFrom.getQValue(&xuix[i].x[0], xuix[i].ui);
    numPoints[xuix[i].ui]++;
  }
  
  kdtrees.resize(num_actions);
  for (unsigned int i=0; i < num_actions; i++) {
    kdtrees[i] = new ANNkd_tree(points[i], numPoints[i], d);
    
    ofstream out("tree.log", fstream::app);
    kdtrees[i]->Print(ANNtrue, out);
    out.close();    
  }
  
  
  { // construct index arrays for next-states in xuix
    nnidx.resize(num_actions);
    dists.resize(num_actions);
    
    for (unsigned int a=0; a < num_actions; a++) {
      int n = numPoints[a];
      if (k < n) n = k;  /// k-nearest neighbour, but don't lookup more neighbours than elements.
      
      nnidx[a] = new int[xuix.size() * n];
      dists[a] = new double[xuix.size() * n];
      
      for (unsigned int p=0; p < xuix.size(); p++) {
        kdtrees[a]->annkSearch(&xuix[p].xnext[0], n, &nnidx[a][n*p], &dists[a][n*p], 0.0); // last arg: precision (0=100%, 0.1 = up too 10% farther away)
      }
    }
  }
  ownPoints = false;
}




KQ::~KQ() {
  for (unsigned int i=0; i < num_actions; i++) {
    delete kdtrees[i];
    if (ownPoints) {
      for (int j=0; j < numPoints[i]; j++) {
        delete [] points[i][j];
      }
    }
    delete [] points[i];
    delete [] values1[i];
    if (values2.size() > i) delete [] values2[i];
    if (nnidx.size() > i) {
      delete [] nnidx[i];
      delete [] dists[i];
    }
  }
  annClose();
}


int KQ::getOptimalAction(const ANNpoint point) const {
  double qvalue = getQValue(point, 0);
  int id = 0;
  for (unsigned int i=1; i < num_actions; i++) {
    double q = getQValue(point, i);
    if (OPT_FIRST_ARG_BETTER(q,qvalue)) {
      qvalue = q;
      id = i;
    }
  }
  return id;
}

static double weightedSum(ANNidxArray idx, ANNdistArray dists, ValueArray values, int k, bool useAutoScaleGaussian) {
  double sum = 0.;
  double sumWeights = 0.;
  
  //  double mean = 0.;
  
  for (int i=0; i < k; i++) {
    double weight;
    if (useAutoScaleGaussian) {
      double sigma = dists[k-1] / 2.; // 2*sigma = weitester punkt
      weight = exp(-(dists[i]*dists[i])/(2*sigma*sigma));
    }
    else {
      weight = dists[i] > 0. ? 1. / dists[i] : 1e9;
    }
    
    sum += weight * values[idx[i]];
    sumWeights += weight;
  }
  
  return sum / sumWeights;
}

double KQ::getQValue(const ANNpoint point, int ui) const {
  int n = kdtrees[ui]->nPoints();
  if (k < n) n = k;  /// k-nearest neighbour, but don't lookup more neighbours than elements.
  
  ANNidxArray idx = new int[n];
  ANNdistArray dists = new double[n];
  
  kdtrees[ui]->annkSearch(point, n, idx, dists, 0.0);
  double sum = weightedSum(idx, dists, values()[ui], n, useAutoScaleGaussian);
  delete [] idx;
  delete [] dists;
  
  return sum;
}



double KQ::getQValue(int pidx, int ui) const {
  int n = kdtrees[ui]->nPoints();
  if (k < n) n = k;  /// k-nearest neighbour, but don't lookup more neighbours than elements.
  
  double sum = weightedSum(&nnidx[ui][pidx*n], &dists[ui][pidx*n], values()[ui], n, useAutoScaleGaussian);
  
  return sum;
}

double KQ::getStateValue(int pidx) const {
  double statev = getQValue(pidx, 0);
  for (unsigned int i=1; i < num_actions; i++) {
    double q = getQValue(pidx, i);
    if (OPT_FIRST_ARG_BETTER(q, statev)) statev = q;
  }
  return statev;
}


double KQ::getStateValue(const ANNpoint point) const {
  double statev = getQValue(point, 0);
  for (unsigned int i=1; i < num_actions; i++) {
    double q = getQValue(point, i);
    if (OPT_FIRST_ARG_BETTER(q, statev)) statev = q;
  }
  return statev;
}


std::ostream& KBRL::operator<<(ostream& out, const KQ& kq) {
  out << kq.num_actions << " " << kq.d << " " << kq.k << endl;
  for (unsigned int i=0; i < kq.num_actions; i++) {
    out << kq.numPoints[i] << " ";
  }
  out << endl;
  
  for (unsigned int ui=0; ui < kq.num_actions; ui++) {
    for (int p=0; p < kq.numPoints[ui]; p++) {
      for (int d=0; d < kq.d; d++) {
        out << kq.kdtrees[ui]->thePoints()[p][d] << " ";
      }
      out << kq.values()[ui][p] << endl;
    }
  }
  
  return out;
}

REGISTER_CONTROLLER(KernelBasedRLController, "A variant of the kernel based RL algorithm introduced by Ormoneit and Sen.")
