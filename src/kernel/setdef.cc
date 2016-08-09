/*
clsquare - closed loop simulation system
Copyright (c) 2004, Neuroinformatics Group, Prof. Dr. Martin Riedmiller,
University of Osnabrueck

Author: Roland Hafner

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
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

#include "setdef.h"
#include "str2val.h"
#include "math.h"
#include <iostream>
#include "global.h"

SetDef::SetDef()
{
  ;
}

bool SetDef::parseStr(const char * str_to_parse, int x_dim)
{
  const char *dum = str_to_parse;
  bool multiSubSet=false;
  bool moreToRead =false;

  if (strskip(dum,"(", dum) == 1) multiSubSet=true;

  do {
    Bounds aktSet;
    int resolution[x_dim];

    bool res = true;
    bool ext_res = true;
    int  warn = 0;
    for (int n=0; n<x_dim; n++) {
      MinMax aktBound;
      aktBound.min=-INFINITY;
      aktBound.max=INFINITY;

      res = res & strskip(dum,"[",dum);
      str2val(dum,aktBound.min,dum);
      str2val(dum,aktBound.max,dum);
      res = res & strskip(dum,"]",dum);
      ext_res = strskip(dum,":",dum);

      if (ext_res)
        str2val(dum,resolution[n],dum);
      else
        resolution[n] = 1;

      aktSet.push_back(aktBound);

      if (!res) warn++;
    }
    if (warn > 0) WOUT(10, warn << " errors occurred while parsing string \"" << str_to_parse << "\"");


    int counter[x_dim];
    double stepsizes[x_dim];
    for (int i = 0; i < x_dim; i++) {
      counter[i] = 0;
      stepsizes[i] = (aktSet[i].max - aktSet[i].min) / (double) resolution[i];
    }

    bool exit = false;
    while (!exit) {
      Bounds resultSet;
      for (int i = 0; i < x_dim; i++) {
        MinMax aktBound;
        aktBound.min = aktSet[i].min + (stepsizes[i] * (double) counter[i]);
        aktBound.max = aktSet[i].min + (stepsizes[i] * ((double) counter[i]+1));
        resultSet.push_back(aktBound);
      }
      subsets.push_back(resultSet);

      counter[0]++;
      int j = 0;
      while (counter[j] == resolution[j]) {
        counter[j] = 0;
        j++;
        if (j == x_dim) {
          exit = true;
          break;
        }
        counter[j]++;
      }
    }

    if (multiSubSet) strskip(dum,")", dum);

    if (multiSubSet) moreToRead = strskip(dum,"(", dum);

  } while (multiSubSet && moreToRead);

  return true;
}

bool SetDef::isWithinSet(const double * state, int x_dim)
{

  for (int i=0; i< (int) subsets.size(); i++) {
    bool isInThisSubset=true;

    if (x_dim != (int)subsets[i].size())
      WOUT(10, "Called with x_dim != bounds dim!");
    int h=0;
    while (h < x_dim && h < (int)subsets[i].size()) {
      if ( state[h] < subsets[i][h].min || state[h] > subsets[i][h].max) isInThisSubset=false;
      h++;
    }

    if (isInThisSubset) return true;
  }

  return false;
}

int SetDef::inWhichSet(const double * state,int x_dim)
{
  for (int i=0; i< (int) subsets.size(); i++) {
    bool isInThisSubset=true;

    if (x_dim != (int)subsets[i].size())
      WOUT(10, "Called with x_dim != bounds dim!");
    int h=0;
    while (h < x_dim && h < (int)subsets[i].size()) {
      if ( state[h] < subsets[i][h].min || state[h] > subsets[i][h].max) isInThisSubset=false;
      h++;
    }

    if (isInThisSubset) return i;
  }

  return -1;
}

void SetDef::clearSubsets()
{
  subsets.clear();
}


void SetDef::print(std::ostream & out)
{
  out << "--- begin of subset: ---\n";
  for (int i=0; i< (int) subsets.size(); i++) {
    out << "SUBSET[" << i << "]\n";
    for (int h=0; h<(int)subsets[i].size(); h++) {
      out << "Dim["<<h<<"]: " << subsets[i][h].min << "  " << subsets[i][h].max << "\n";
    }
  }
  out << "--- end of subset. ---\n";
}
