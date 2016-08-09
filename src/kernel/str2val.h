/*
 * Copyright (c) 1999 - 2001, Artur Merke <artur.merke@udo.edu> 
 *

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

#ifndef _STR2VAL_H_
#define _STR2VAL_H_

int strskip(char const* str,char chr, char const* & next);
inline int strskip(char const* str,char chr) {
  const char * dum;
  return strskip(str,chr,dum);
}

/*skips just the expression, regardless of what follows in str, this is
 different to the earlier version of strskip */
int strskip(char const* str,char const* expr, char const* & next);
inline int strskip(char const* str,char const* expr) {
  const char * dum;
  return strskip(str,expr,dum);
}

/*tests if this string contains just white space until ist end*/
int strempty(char const* str, char const* & next);
inline int strempty(char const* str) {
  const char * dum;
  return strempty(str,dum);
}

/*tests if this string contains at least 1 white space char at its begin*/
int strspace(char const* str, char const* & next);
int strspace(char const* str);

/*the value of the specific type is recognized as far as possible*/
int str2val(char const* str,    int & val, char const* & next);
int str2val(char const* str,   long & val, char const* & next);
int str2val(char const* str, double & val, char const* & next);
int str2val(char const* str,  float & val, char const* & next);
int str2val(char const* str,   bool & val, char const* & next);

inline int str2val(char const* str,    int & val) {
  const char * dum;
  return str2val(str, val, dum);
}
inline int str2val(char const* str,   long & val) {
  const char * dum;
  return str2val(str, val, dum);
}
inline int str2val(char const* str, double & val) {
  const char * dum;
  return str2val(str, val, dum);
}
inline int str2val(char const* str,  float & val) {
  const char * dum;
  return str2val(str, val, dum);
}
inline int str2val(char const* str,   bool & val) {
  const char * dum;
  return str2val(str, val, dum);
}

int str2val(char const* str, int num,    int * val, char const* & next);
int str2val(char const* str, int num,   long * val, char const* & next);
int str2val(char const* str, int num, double * val, char const* & next);
int str2val(char const* str, int num,  float * val, char const* & next);
int str2val(char const* str, int num,   bool * val, char const* & next);

inline int str2val(char const* str, int num,    int * val) {
  const char * dum;
  return str2val(str, num, val, dum);
}
inline int str2val(char const* str, int num,   long * val) {
  const char * dum;
  return str2val(str, num, val, dum);
}
inline int str2val(char const* str, int num, double * val) {
  const char * dum;
  return str2val(str, num, val, dum);
}
inline int str2val(char const* str, int num,  float * val) {
  const char * dum;
  return str2val(str, num, val, dum);
}
inline int str2val(char const* str, int num,   bool * val) {
  const char * dum;
  return str2val(str, num, val, dum);
}

#endif
