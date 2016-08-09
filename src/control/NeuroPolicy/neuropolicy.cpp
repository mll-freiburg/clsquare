/*
clsquare - closed loop simulation system
Copyright (c) 2010-2012 Machine Learning Lab, 
Prof. Dr. Martin Riedmiller, University of Freiburg

Author: Thomas Lampe

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

#include "neuropolicy.h"
#include "valueparser.h"

bool NeuroPolicy::get_action(const double* observed_state, double* action)
{
  for (_i=0; _i<_sdim; _i++)
    _net->in_vec[_i] = observed_state[_i];
  _net->forward_pass(_net->in_vec, _net->out_vec);
  for (_i=0; _i<_adim; _i++)
    action[_i] = _net->out_vec[_i];
  return true;
}

bool NeuroPolicy::init(const int observation_dim, const int action_dim, double deltat, const char* fname, const char* chapter)
{
  ValueParser vp(fname, chapter==0?"Controller":chapter);

  char filename[255];
  if (vp.get("filename", filename, 255) < 1)
    EOUT("No filename specified.");
  _net = new Net();
  _net->load_net(filename);

  _adim = action_dim;
  _sdim = observation_dim;

  return true;
}

REGISTER_CONTROLLER(NeuroPolicy, "A simple controller to play policies saved as neural networks.")
