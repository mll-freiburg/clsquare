
/*
n++: neural network simulator (multi-layer-perceptrons)

Copyright (c) 1994, 2004, Martin Riedmiller, University of Osnabrueck
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

/******************************************************************************/

/* N++ : C++ class neural network simulator                                   */
/* (c) 1994 Martin Riedmiller                                                 */
/* last changed: 27.7.94                                                      */

/* File: n++.h                                                                */
/* Purpose: n++ header file - to be included in application programs          */

/******************************************************************************/

#include<stdio.h>
#include<math.h>
#include<string.h>
#include<stdlib.h>

#ifndef NPP
#define NPP

#define ABSO(x) (x>0?x:-x)

/* EXTERNAL DECLARATIONS: */
#define FTYPE double       /* computing precision */
#define MAX_PARAMS 10     /* max. no of parameters for weight update,... */

/* Identification number for the functions (see include file "functions.c") */
/* Activation functions */
#define LOGISTIC 0
#define SYMMETRIC 1
#define LINEAR 2
/* Update functions */
#define BP 0
#define RPROP 1

/* Error Codes */
#define OK 0
#define NPP_ERROR -1
#define FILE_ERROR -2

/* INTERNAL DECLARATIONS */
#define MAX_VARIABLES 3   /* number of user's variables in weight/unit structure */
#define MAX_LAYERS 100    /* max. no of layers */
#define BIAS 0       /* unit[0] is bias pseudo unit with output 1 */
/* Macros */
#define IS_INPUT(i) (i<=topo_data.in_count)
#define IS_UNIT(i) ((i>0)&&(i<=topo_data.unit_count))
#define IS_LAYER(l) ((l>=0)&&(l<topo_data.layer_count))

#ifdef MSDOS
/* fake drand48 using ansi lib and a macro */
#define drand48 (1.0/((double) RAND_MAX)) *rand
#define srand48(seed) srand((unsigned int) (seed))
#else
extern "C" double drand48( void ); 
extern "C" void srand48( long ); 
#endif

#define RESERVOIR 1
#define FEEDFORWARD 0

struct weight_typ{
  int from_unit;          /* no of incoming unit: -1 == bias unit */
  weight_typ *next; 
  FTYPE value;	
  FTYPE delta;
  FTYPE dEdw;
  FTYPE dEdw_sequence;    /* TD learning */
  FTYPE dOdw;             /* TD learning */
  FTYPE variable[MAX_VARIABLES]; /* for programmer's use */  
};

struct unit_typ{
  int unit_no;                   /* position of unit in unit array */
  weight_typ *weights;
  FTYPE dEdo;
  FTYPE dEdnet;                 
  FTYPE out;                     /* output_value */
  FTYPE net_in;                  /* summed input */
  FTYPE new_out;                  /* used in reservoir nets */
  FTYPE variable[MAX_VARIABLES]; /* programmer's use */
  FTYPE (*act_f)(unit_typ*);     /* pointer to activation function */
  FTYPE (*deriv_f)(unit_typ*);   /* pointer to derivation of activation function */
  unsigned short act_id;         /* identification number of activation function */
};			

struct layer_typ{
  unit_typ *unit;
  int unit_count;
};

struct topo_typ{
  int layer_count;  /* number of layers */
  int encoder_layer; // assumed encoder layer for deep encoder networks
  int in_count,out_count,hidden_count,unit_count, encoder_count;   /* number of rsp. units */
};

struct scale_typ{
  int scale_function;
  int position;
  double parameter1;
  double parameter2;
  double parameter3;
  scale_typ *next;
};

class Net{
 protected:
  int network_type;
  /* net data structure */
  unit_typ *unit;   /* array of all units in network */
  layer_typ layer[MAX_LAYERS];   /* virtual segmentation in layers */
  double update_params[MAX_PARAMS]; /* parameters for weight update function */
  scale_typ *scale_list_in, *scale_list_out;
  FTYPE *scaled_in_vec;
  void (*forward_pass_f)(unit_typ*,topo_typ*);  /* pointer to fw pass function */
  void (*backward_pass_f)(unit_typ*,topo_typ*); /* pointer to bw pass function */
  void (*update_f)(unit_typ*,topo_typ*,double*); /* pointer to update function */
  void create_units();
  void insert_scale_element(scale_typ **scale_list,int position,
			    int scale_function, double param1,double param2,double param3);
  void apply_scaling(double* data_vector, scale_typ *scale_list);
  void apply_backward_scaling(double* data_vector, scale_typ *scale_list);
  void delete_structure();
 public:
  topo_typ topo_data;
  int update_id;             /* identity of update function */
  FTYPE *in_vec, *out_vec, *encoder_vec;   /* communication vectors (have the appropriate size) */
  Net(void);
  ~Net(); 
  int create_layers(int layers, int layer_nodes[]);
  void connect_units(int to_unit, int from_unit, FTYPE value);
  void connect_reservoir(double connectivity_rate);
  void connect_layers();
  void connect_shortcut();
  void set_unit_act_f(int unit_no,int act_id);
  void set_layer_act_f(int layer_no,int act_id);
  int set_update_f(int typ,double *params);
  int save_net(char filename[]);
  int load_net( char filename[]);
  void print_net();
  void print_net_activations();

  void set_seed(long seed_no);
  void init_weights(int mode, FTYPE range);
  void reset_reservoir_neurons();
  void forward_pass(FTYPE *in_vec, FTYPE *out_vec);
  void forward_pass_encoder(FTYPE *in_vec, FTYPE *out_vec);
  void backward_pass(FTYPE *dedout, FTYPE *dedin);
  void backward_pass_light(FTYPE *dedout, FTYPE *dedin);
  void TD_backward_pass(FTYPE td_error, double lambda);
  void TD_init_sequence();
  void update_weights();
  void clear_derivatives();
  void multiply_derivatives(double factor);
  void TD_terminate_sequence(double weighting_factor);
};

#endif //NPP
