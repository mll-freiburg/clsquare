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

#include "neuroobserver.h"
#include "valueparser.h"

bool NeuroObserver::init (const int plant_state_dim, const int measurement_dim, const int action_dim, int &observed_state_dim, const char *fname, const char *chapter)
{
  ValueParser vp(fname, chapter==NULL ? "Observer" : chapter);

  // load net
  _filename = new char[255];
  if (vp.get("filename", _filename, 255) < 1) {
    EOUT("No filename specified.");
    return false;
  }
  _net.load_net(_filename);
  vp.get("reload", _reload, false);

  // dimensions
  _idim = _net.topo_data.in_count;
  _odim = _net.topo_data.out_count;
  _mdim = measurement_dim;
  observed_state_dim = _mdim + _odim;

  // determine dimension selection
  _assign = new int[_idim];
  for (_i=0; _i<_idim; _i++)
    _assign[_i] = _i;
  vp.get("assignment", _assign, _idim);
  for (_i=0; _i<_idim; _i++)
    if (_assign[_i] >= measurement_dim) {
      EOUT("Input dimension (" << _assign[_i] << ") out of range (" << measurement_dim << ")");
      return false;
  }

  return true;
}

void NeuroObserver::get_observed_state (const double *prev_measurement, const double* prev_action, const double *current_measurement, const int cycle_ctr, double *observed_state)
{
  for (_i=0; _i<_mdim; _i++)
    observed_state[_i] = current_measurement[_i];
  for (_i=0; _i<_idim; _i++)
    _net.in_vec[_i] = current_measurement[_assign[_i]];
  _net.forward_pass(_net.in_vec, _net.out_vec);
  for (_i=0; _i<_odim; _i++)
    observed_state[_mdim+_i] = _net.out_vec[_i];

}

void NeuroObserver::notify_episode_starts ()
{
  if (_reload)
    _net.load_net(_filename);
}

REGISTER_OBSERVER(NeuroObserver, "Uses a neural network to generate an observation from parts of the system state.");

