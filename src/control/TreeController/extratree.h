/*
clsquare - closed loop simulation system
Copyright (c) 2010, 2011, 2012 Machine Learning Lab, 
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

#ifndef __EXTRATREE_H__
#define __EXTRATREE_H__

#include <pthread.h>
#include <vector>
#include <iostream>
#include <fstream>

// definitions in this file
class  ExtraTree;
class  ExtraTreeNode;
struct ExtraTreeWorkerData;
struct ExtraTreePattern;
struct ExtraTreeAttribute;

/** Implements Extra trees for supervised regression problems.
 *  The algorithm follows Geurts et. al (2004): 
 *  Extremely randomized trees, a tree-based ensemble method for supervised 
 *  regression problems. It essentially consists of randomizing strongly both 
 *  attribute and cut-point choice while splitting a tree node. In the extreme 
 *  case, it builds totally randomized trees whose structures are independent 
 *  of the output values of the learning sample. The strength of the 
 *  randomization can be tuned to problem specifics. 
 *  @author Manuel Blum
 */
class ExtraTree
{
public:
	/** Construct an empty ExtraTree for given input dimensionality. 
	 *  @param input_dim input dimensionality */
	ExtraTree (int input_dim);	
	/** Load and ExtraTree from file.
	 *  @param filename filename of previously constructed ExtraTree */
	ExtraTree (const char* filename);	
	/** Destructor. */
	virtual ~ExtraTree ();	
	/** Add input x and target y to patternset. 
	 *  @param x input vector
	 *  @param y target value */
	void add_pattern(const double x[], double y);	
	/** Delete all patterns from sampleset. */
	void clear_sampleset();
	/** Learn an Extra tree ensemble, given the data in the patternset. 
	 *  @param m number of trees in the ensemble
	 *  @param k number of dimensions to check for split point, if k=1 
	 *           the tree corresponds to a completly randomized tree
	 *  @param n_min nodes containing less than n_min samples are terminal*/
	void train( int m, int k, int n_min);
	/** Learn an Extra tree ensemble, given the data in the patternset. 
	 *  @param m number of trees in the ensemble
	 *  @param k number of dimensions to check for split point, if k=1 
	 *           the tree corresponds to a completly randomized tree
	 *  @param n_min nodes containing less than n_min samples are terminal 
   *  @param threads number of threads*/
	void train( int m, int k, int n_min, int threads);
	double predict(const double* x);  ///< Predict vector x.
  /**  Save tree ensemble to file.
   *   first value is the input dimensionality
   *   a new tree is preceded by -1 */
	void write(const char* filename);
	bool read(const char* filename);  ///< Load tree ensemble from file.
	int num_trees(); ///< Returns the number of trees in the ensemble
	std::string to_string();          ///< Returns a string representation of the ensemble.
  /** Called by each thread with the thread's individual set of parameters stored in arg. */
  static void* parallel_build_tree(void* arg); 
  /** Check if target values in sample set are constant. */
  static bool is_constant(std::vector<ExtraTreePattern> &samples);
  static bool is_constant(std::vector<ExtraTreePattern> &samples, int start, int end);
  /** Split dataset according to split in node. */
  static int split(std::vector<ExtraTreePattern> &samples, ExtraTreeNode *node, int start, int end);
  static double random(double floor, double ceil, unsigned int *seed);
private:
  /** Builds a regression tree from samples and returns its root node. */
  ExtraTreeNode* build_tree(ExtraTreeWorkerData *data, int start, int end);
  /** Return test for samples according to parameter k. */
  bool get_test(ExtraTreeWorkerData *data, ExtraTreeNode *node, int start, int end);
  /** Return score of given test. */	
  double get_score(std::vector<ExtraTreePattern> &samples, int t, double threshold, int start, int end);
	/** Read node from filestream. */
	ExtraTreeNode* read_node(std::ifstream &fs);
	/** Writes the given node and all of its child nodes to fs. 
   *   nonterminal nodes: (dimension) (test) (left child) (right child)
   *   terminal nodes:    -1 (function value) */
	void write_node(ExtraTreeNode* node, std::ofstream &fs);
	/** The sample set. */
	std::vector<ExtraTreePattern> samples;
	/** The root nodes of the tree ensemble. */
	std::vector<ExtraTreeNode*> ensemble;
	/** Input-space dimensionality of the regression problem. */
	int input_dim;
  /** Returns a string representation of of the given node. */
	std::string node2str(ExtraTreeNode* node, int* n);
};

/** Training pattern consisting of input vector and target value. */
struct ExtraTreePattern
{
  double* input;
  double output;
};

struct ExtraTreeAttribute
{
  double min;
  double max;
  int index;
};

/** Represents a node in an ExtraTree. */
class ExtraTreeNode
{
public:
	ExtraTreeNode () 
  {
    test = -1;
    value = 0.0;
    left = right = NULL;
  }
	virtual ~ExtraTreeNode ()
  {
    delete left;
    delete right;
    left = right = NULL;
  };
	/** Position of the split in dimension test or prediction value if leaf node. */
	double value;
	int test;                ///< Dimension of the split, -1 for a leaf node.
	ExtraTreeNode* left;     ///< Child node.
	ExtraTreeNode* right;    ///< Other child node.
};

struct ExtraTreeWorkerData 
{
  pthread_t thread_id;
  size_t thread;
  size_t num_threads;
  unsigned int seed;
  size_t k;
  size_t n_min;
  std::vector<ExtraTreePattern> samples;
  ExtraTree *tree;
};

#endif /* __EXTRATREE_H__ */
