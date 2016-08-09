/*
clsquare - closed loop simulation system
Copyright (c) 2004, Neuroinformatics Group, Prof. Dr. Martin Riedmiller,
University of Osnabrueck

Author: Roland Hafner, Stephan Timmer

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

#ifndef _SET_DEF_H_
#define _SET_DEF_H_

#include <vector>
#include <iostream>

/** Implements a datatype for representings subsets \f$ S \subset R^n \f$. A set is given
 *  by a union of subsets (represented as intervals in every dimension). See below for exact syntax. 
 * \author Roland Hafner, Stephan Timmer*/
class SetDef {
 public:
  /** Startpoint and Endingpoint of an interval */
  typedef struct{
    double min, max;
  } MinMax;

  /** Set of intervals, that forms a subset of \f$  R^n \f$ */
  typedef std::vector<MinMax> Bounds;

  /** A set is given by a union of several subsets */
  std::vector<Bounds> subsets;
  
  /** Construct an empty set. */
  SetDef();

  /** Parse a set from string.
   * \param str_to_parse String to parse.
   * \param x_dim Dimension (n for \f$ R^n \f$) 
   * \return true, for success 
   * 
   *  Syntax:   
   *  "([ min max ][ min max][ min max]...x_dim)([ min max][ min max][ min max]...x_dim)(...)" -> 
   *  several subsets. 
   *  It is also possible to add a resolution to an interval: [min max]:resolution. 
   *  Resolution x divides the interval into x intervals of equal length. 
   *  Example:   
   *  "([1 4]:2 [5 10]:2])" gives the set: "([1 2] [5 7.5])([1 2] [7.5 10])([2 4] [5 7.5])([2 4] [7.5 10])" */
  bool parseStr(const char * str_to_parse, int x_dim);

  /** Checks, if a subset contains a certain point. 
   * \param state Point in space.
   * \param x_dim Dimension of space.
   * \param true if point is part of the space */
  bool isWithinSet(const double * state, int x_dim);

  /** Computes subset, which contains a certain point. 
   * \param state Point in space.
   * \param x_dim Dimension of space.
   * \param index of subset (-1, if no subset is found) */
  int inWhichSet(const double * state,int x_dim);


  /** Deletes all subsets. */
  void clearSubsets(); 

  /** Gives the current amount of subsets.
   \return number of subsets.*/
  int  numSubsets(){return subsets.size();}

  /** Prints the whole set (with all subsets). 
   \param out Output stream. */
  void print(std::ostream & out);
};

#endif
