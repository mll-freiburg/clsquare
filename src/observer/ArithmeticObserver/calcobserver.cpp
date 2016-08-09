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

#include "calcobserver.h"
#include "valueparser.h"
#include <stdio.h>
#include <cstdlib>
#include <cstring>
#include <cmath>

#define SSSET(xxx) { ss.str(""); ss << xxx; };

bool ArithmeticObserver::init (const int plant_state_dim, const int measurement_dim, const int action_dim, int &observed_state_dim, const char *fname, const char *chapter)
{
  ValueParser vp(fname, chapter==NULL ? "Observer" : chapter);
  std::stringstream ss;
  _mdim = observed_state_dim = measurement_dim;

  int len;
  char *tmp;
  for (_i=0; ; _i++) {

    // try to get next controller name
    tmp = new char[255];
    SSSET("operation_" << _i);
    len = vp.get(ss.str().c_str(), tmp, 255);
    if (len < 1) break;

    // search for terms
    std::vector<int> inds;
    for (_k=0; _k<len; _k++) {
      if (tmp[_k] == ' ')
        tmp[_k] = '\n';
      else if (_k == 0 || tmp[_k-1] == '\n')
        inds.push_back(_k);
    }

    // check sanity
    if (inds.size() < 1) break;
    if (inds.size() % 2 == 0)
      WOUT(10, "Expression " << _i << " contains an even number of elements. Last one will be ignored (" << &tmp[inds[inds.size()-1]] << ").");

    // build expression structure
    std::vector<ArobsOperation> expr;
    expr.push_back(ArobsOperation('+', &tmp[inds[0]], observed_state_dim));
    for (_k=1; _k<(int)inds.size()-1; _k+=2)
      expr.push_back(ArobsOperation(tmp[inds[_k]], &tmp[inds[_k+1]], observed_state_dim));
    _ops.push_back(expr);
    observed_state_dim++;

    // re-convert and output as validation
    SSSET("Added expression: ");
    for (_k=0; _k<(int)expr.size(); _k++)
      expr[_k].print(ss);
    IOUT(ss.str().c_str());
  }

  return true;
}

void ArithmeticObserver::get_observed_state (const double *prev_measurement, const double* prev_action, const double *current_measurement, const int cycle_ctr, double *observed_state)
{
  for (_i=0; _i<_mdim; _i++)
    observed_state[_i] = current_measurement[_i];

  for (_i=0; _i<(int)_ops.size(); _i++) {
    observed_state[_i+_mdim] = 0.;
    for (_k=0; _k<(int)_ops[_i].size(); _k++)
      observed_state[_i+_mdim] = _ops[_i][_k].eval(observed_state[_i+_mdim], observed_state);
  }
}

double ArobsOperation::eval (const double base, const double* state)
{
  switch (_op) {
    case '+': return base + _nm.eval(state);
    case '-': return base - _nm.eval(state);
    case '*': return base * _nm.eval(state);
    case '/': return base / _nm.eval(state);
    case '^': return pow(base, _nm.eval(state));
    case '%': return fmod(base, _nm.eval(state));
    case 's': return base < 0 ? asin(_nm.eval(state)) : sin(_nm.eval(state));
    case 'c': return base < 0 ? acos(_nm.eval(state)) : cos(_nm.eval(state));
    default:
      EOUT("Operator " << _op << " not known.");
  }
  return INFINITY;
}

void ArobsOperand::set (const char* desc, const int dim)
{
  _ob = desc[0] == '$';
  _nm = atof(&desc[_ob?1:0]);
  if (_ob && int(_nm) >= dim)
    WOUT(10, "Measurement dimension " << int(_nm) << " will not be defined when calculating dimension " << dim << ". Undefined behaviour may result.");
}

REGISTER_OBSERVER(ArithmeticObserver, "Observer that performs basic arithmetic operations on the measurement.");

