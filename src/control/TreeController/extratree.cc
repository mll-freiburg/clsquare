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

#include <cmath>
#include <limits>
#include <stdlib.h>
#include <pthread.h>
#include <assert.h>
#include <sstream>
#include <algorithm>
#include <cstring>

#include "extratree.h"
#include "onlinestat.h"

#define DEFAULT_NUM_THREADS 8

#ifdef XCode
#pragma mark -
#pragma mark build tree
#endif

static const double rand_max = double(RAND_MAX)+2.0;
static const double epsilon  = 1e-6;

void* ExtraTree::parallel_build_tree(void* arg) 
{
  // arguments of the individual thread
  ExtraTreeWorkerData* data = static_cast<ExtraTreeWorkerData*>(arg);  
  std::vector<ExtraTreeNode*> *ensemble = &data->tree->ensemble;
  for (int i = 0; i < (int)ensemble->size(); ++i) {
    if ( i % data->num_threads == data->thread) {
      ensemble->at(i) = data->tree->build_tree(data, 0, data->samples.size()); 
    }
  }  
  return 0;
}

ExtraTreeNode* ExtraTree::build_tree(ExtraTreeWorkerData *data, int start, int end) 
{	
  assert(start < end);
  assert(start >= 0);
  assert(end <= (int)data->samples.size());
  ExtraTreeNode* node = new ExtraTreeNode();
  if (get_test(data, node, start, end)) {
		// split sample set according to test
    int s = split(data->samples, node, start, end);
		// build subtrees and add to child nodes
    assert(start < s);
    assert(s < end);
		node->left = build_tree(data, start, s);
		node->right = build_tree(data, s, end);
  } else {
    // compute average value of sample set
		node->value = 0.0;
		for(int i = start; i<end; ++i) node->value += data->samples[i].output;
		node->value /= (end-start);
  }
  return node;
}

int ExtraTree::split(std::vector<ExtraTreePattern> &samples, ExtraTreeNode *node, int start, int end)
{
  int high = end-1;
  int low = start;  
  while (low < high) {
    while (low < end-1 && samples[low].input[node->test] <= node->value) low++;
    while (high > start && samples[high].input[node->test] >= node->value) high--;
    if (low < high) std::swap(samples[low], samples[high]);
  } 
  return low;
}

bool ExtraTree::is_constant(std::vector<ExtraTreePattern> &samples)
{
  return ExtraTree::is_constant(samples, 0, samples.size());
}

bool ExtraTree::is_constant(std::vector<ExtraTreePattern> &samples, int start, int end)
{
	double maximum = -INFINITY, minimum = INFINITY;
  std::vector<ExtraTreePattern>::iterator it;
  for(it = samples.begin()+start; it!=samples.begin()+end; ++it) {
    if (it->output > maximum) maximum = it->output;
    if (it->output < minimum) minimum = it->output;
    if (maximum-minimum > epsilon) return false;
  }
	return true;
}

bool ExtraTree::get_test(ExtraTreeWorkerData *data, ExtraTreeNode *node, int start, int end) {
  // dataset is sufficiently small
  if (end - start < (int)data->n_min) return 0;
  // dataset has constant target
  if (ExtraTree::is_constant(data->samples, start, end)) return 0;
  std::vector<ExtraTreeAttribute> attributes;
  for (int i=0; i<data->tree->input_dim; ++i) {
    ExtraTreeAttribute a;
    // compute minimum and maximum of attribute
    a.min = INFINITY;
    a.max = -INFINITY;
    a.index = i;
    std::vector<ExtraTreePattern>::iterator it;
    for(it = data->samples.begin()+start; it!=data->samples.begin()+end; ++it) {
      if (it->input[i] > a.max) a.max = it->input[i];
      if (it->input[i] < a.min) a.min = it->input[i];
    }
    if (a.max-a.min > epsilon) attributes.push_back(a);    
  }
  // check if all attributes are constant
  if (attributes.size()==0) return 0;
  // randomize attributes 
  std::random_shuffle(attributes.begin(), attributes.end());
  // check if there is just one attribute to consider
  if (attributes.size()==1 || data->k < 2) {
    ExtraTreeAttribute *a = &attributes[0];
    node->value = random(a->min, a->max, &data->seed);
    node->test = a->index;
    return true;
  }
  int k = data->k > attributes.size() ? attributes.size() : data->k;
  double best = -INFINITY, score, threshold;
  for (int i=0; i<k; ++i) {
    ExtraTreeAttribute *a = &attributes[i];
    // draw random threshold
    threshold = random(a->min, a->max, &data->seed);
    score = get_score(data->samples, a->index, threshold, start, end);
    if (score > best) {
      best = score;
      node->test = a->index;
      node->value = threshold;
    }
  }  
  return true;
}

double ExtraTree::random(double floor, double ceil, unsigned int *seed)
{
  return (ceil - floor) * (rand_r(seed)+1.0)/rand_max + floor;
}


double ExtraTree::get_score(std::vector<ExtraTreePattern> &samples, int test, double value, int start, int end) 
{
	// compute variance in this node and both child nodes
	OnlineStat stat_all;
	OnlineStat stat_left;
	OnlineStat stat_right;
	int n = end - start;
	for(int i=start; i<end; ++i) {
    stat_all.append(samples[i].output);
		if (samples[i].input[test] < value) {
			stat_left.append(samples[i].output);
		} else {
			stat_right.append(samples[i].output);		
		}
	}
	double p = stat_left.get_n_times_var()/n;
	double q = stat_right.get_n_times_var()/n;
	return (stat_all.get_var() - p - q) / stat_all.get_var();
}

#ifdef XCode
#pragma mark -
#pragma mark constructor/destructor
#endif

ExtraTree::ExtraTree(int input_dim)
{
	this->input_dim = input_dim;
}

ExtraTree::ExtraTree(const char* filename)
{
	this->read(filename);
}

ExtraTree::~ExtraTree()
{
	// delete pattern set
	clear_sampleset();
	// delete tree ensemble	
	while(!ensemble.empty()) {
		delete ensemble.back();
		ensemble.pop_back();
	}
}

#ifdef XCode
#pragma mark -
#pragma mark data
#endif

int ExtraTree::num_trees()
{
  return ensemble.size();
}

void ExtraTree::add_pattern(const double x[], double y)
{
	ExtraTreePattern pattern;
	pattern.input = new double[input_dim];
  memcpy(pattern.input, x, sizeof(double)*input_dim);
	samples.push_back(pattern);
	samples.back().output = y;
}


void ExtraTree::clear_sampleset() {
	while(!samples.empty()) {
		delete [] samples.back().input;
		samples.pop_back();
	}
}

#ifdef XCode
#pragma mark -
#pragma mark train/predict
#endif

void ExtraTree::train(int m, int k, int n_min) 
{
  train(m, k, n_min, DEFAULT_NUM_THREADS);
}

void ExtraTree::train(int m, int k, int n_min, int num_threads)
{
  assert(n_min>2);
	if (samples.size() < 2) {
		std::cerr << "error: no data" << std::endl;
		return;
	}
	if (k > input_dim) k = input_dim;
  if (k < 1) k = 1;
	// delete tree ensemble	
	while(!ensemble.empty()) {
		delete ensemble.back();
		ensemble.pop_back();
	}
  ensemble.resize(m, 0);
  // use at least one thread
  num_threads = num_threads < 1 ? 1 : num_threads;
  // use at most as many threads as trees in the ensemble
  num_threads = num_threads > (int)ensemble.size() ? ensemble.size() : num_threads;
  // vector will hold the individual parameters of each thread
  std::vector<ExtraTreeWorkerData> worker_data(num_threads);
	// train ensemble
  for (int i=0; i < num_threads; i++) {
    worker_data[i].thread = i;
    worker_data[i].num_threads = num_threads;
    worker_data[i].seed = time(NULL);
    worker_data[i].samples = samples;
    worker_data[i].tree = this;
    worker_data[i].k = k;
    worker_data[i].n_min = n_min;
    // boss joins in 
    if (i == num_threads-1) parallel_build_tree(&worker_data[i]);
    // create worker
    else {
      int err = pthread_create(&(worker_data[i].thread_id), 0, parallel_build_tree, (void*) &worker_data[i]);
      if (err) std::cerr << "Error creating pthread " << err << std::endl;
    }
  }
  // wait for all threads to finish their work
  for (int i=0; i < num_threads-1; i++) {
    int err = pthread_join(worker_data[i].thread_id, 0);
    if (err) std::cerr << "Error joining pthread " << err << std::endl;
  }
}

double ExtraTree::predict(const double* x) 
{
  if (ensemble.empty()) return 0.0;
  double y = 0.0;
  std::vector<ExtraTreeNode*>::iterator i;
  ExtraTreeNode* node;
  for (i = ensemble.begin(); i!=ensemble.end(); ++i) {
    node = *i;
    while(node->test != -1) {
      if (x[node->test]<node->value) node = node->left;
      else node = node->right;
    }
    y += node->value;
  }
  return y / ensemble.size();
}

#ifdef XCode
#pragma mark -
#pragma mark read/write
#endif

void ExtraTree::write(const char* filename)
{
  std::ofstream fs(filename, std::ios::out | std::ios::binary);
  fs.write((char*)&input_dim, sizeof(int));
	std::vector<ExtraTreeNode*>::iterator i;
  int n = -1;
	for(i = ensemble.begin(); i != ensemble.end(); ++i) {
    fs.write((char*)&n, sizeof(int));
    write_node(*i, fs);
	}
	fs.close();
}

void ExtraTree::write_node(ExtraTreeNode* node, std::ofstream &fs)
{
  fs.write((char*)&(node->test), sizeof(int));
  fs.write((char*)&(node->value), sizeof(double));
	if (node->test != -1) {
    write_node(node->left, fs);
    write_node(node->right, fs);
	}
}

bool ExtraTree::read(const char* filename)
{
  std::ifstream fs(filename, std::ios::in | std::ios::binary);
	// delete tree ensemble	
	while(!ensemble.empty()) {
		delete ensemble.back();
		ensemble.pop_back();
	}
	if (fs.good()) {
    fs.read((char*) &input_dim, sizeof(int));
  } else {
    std::cerr << "error reading file " << filename << std::endl;
    fs.close();
    return false;
	} 
  while (fs.good()) {
    int n=0;
    fs.read((char*) &n, sizeof(int));
    if (n!=-1) break;
    ensemble.push_back(read_node(fs));
  } 
	fs.close();
	return true;
}

ExtraTreeNode* ExtraTree::read_node(std::ifstream &fs)
{
	ExtraTreeNode* node = new ExtraTreeNode();
  fs.read((char*)&(node->test), sizeof(int));
  fs.read((char*)&(node->value), sizeof(double));
	if (node->test != -1) {
		node->left = read_node(fs);
		node->right = read_node(fs);
	}
	return node;	
}

std::string ExtraTree::to_string() 
{
	std::ostringstream s;
	s << "# regression tree ensemble" << std::endl << "#" << std::endl
  << "# different trees are separated by a blank line" << std::endl
  << "# each line corresponds to one node" << std::endl << "#" << std::endl
  << "# nonterminal nodes: (index) (dimension) (test) (left child) (right child)" 
  << std::endl << "# terminal nodes:    (index) -1 (function value)" 
  << std::endl << std::endl << "input_dim " << input_dim << std::endl;
	
	std::vector<ExtraTreeNode*>::iterator i;
	int j = 0;
	for(i = ensemble.begin(); i != ensemble.end(); ++i) {
		s << std::endl << "# Tree " << ++j << ":" << std::endl;
		int n = 0;
		s << node2str(*i, &n) << std::endl;
	}
	return s.str();
}

std::string ExtraTree::node2str(ExtraTreeNode* node, int* n_ptr)
{
	std::ostringstream s;
	s << *n_ptr << " " << node->test << " " << node->value;
	
	if (node->test != -1) {
		s << " " << ++(*n_ptr);
		std::string left = node2str(node->left, n_ptr);
		s << " " << ++(*n_ptr);
		std::string right = node2str(node->right, n_ptr);
		s << std::endl << left << right;
	} else {
		s << std::endl;
	}
	return s.str();
}
