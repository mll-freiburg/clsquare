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

#include "str2val.h"
#include <stdlib.h>
#include <errno.h>
#include <limits.h>

#define IS_WHITE_CHAR(x) ( ' ' == (x) || '\t' == (x) || '\n'== (x) )
#define IS_STR_END_CHAR(x) ( '\0' == (x) )

int strskip(char const* str,char chr, char const* & next) {
  if (0==str) {
    next= 0;
    return 0;
  }

  while( IS_WHITE_CHAR(*str) )
    str++;
  
  next= str;
  if ( *str != chr )
    return 0;

  next= str+1; 
  return 1;
}

int strskip(char const* str,char const* expr, char const* & next) {
  if (0==str || 0==expr) {
    next= 0;
    return 0;
  }

  while( IS_WHITE_CHAR(*str) )
    str++;
  
  while ( *str == *expr && !IS_STR_END_CHAR(*expr) ) {
    str++;
    expr++;
  }

  next= str; 
  
  if ( !IS_STR_END_CHAR(*expr) ) 
    return 0;

  return 1;
}

int strempty(char const* str, char const* & next) {
  if (0==str) {
    next= 0;
    return 1;
  }

  while( IS_WHITE_CHAR(*str) )
    str++;
  
  next= str;
  if ( IS_STR_END_CHAR(*str) ) 
    return 1;

  return 0;
}

int strspace(char const* str, char const* & next) {
  if (0==str) {
    next= 0;
    return 0;
  }

  if ( ! IS_WHITE_CHAR(*str) ) {
    next= str;
    return 0;
  }

  while( IS_WHITE_CHAR(*str) )
    str++;
  
  next= str;

  return 1;
}

int strspace(char const* str) {
  if (0==str) 
    return 0;
  return IS_WHITE_CHAR(*str);
}

int str2val(char const* str,int & val, char const* & next) {
  char * error_ptr;
  long tmp_long;

  if (0 == str) {
    next= 0;
    return 0;
  }

  tmp_long= strtol(str, &error_ptr, 10);

  if ( str == error_ptr 
       || ERANGE == errno 
       || tmp_long > INT_MAX || tmp_long < INT_MIN ) {
    next= error_ptr;
    return 0;
  }

  val= tmp_long;
  next= error_ptr;
  
  return 1;
}

int str2val(char const* str,long & val, char const* & next) {
  char* error_ptr;
  long tmp_long;

  if (0 == str) {
    next= 0;
    return 0;
  }

  tmp_long= strtol(str, &error_ptr, 10);

  if ( str == error_ptr
       || ERANGE == errno 
       || tmp_long > LONG_MAX || tmp_long < LONG_MIN ) {
    next= error_ptr;
    return 0;
  }

  val= tmp_long;
  next= error_ptr;

  return 1;
}

int str2val(char const* str,double & val, char const* & next) {
  char* error_ptr;
  double tmp_double;

  if (0 == str) {
    next= 0;
    return 0;
  }

  tmp_double= strtod(str, &error_ptr);

  if ( str == error_ptr 
       || ERANGE == errno ) {
    next= error_ptr;
    return 0;
  }

  val= tmp_double;
  next= error_ptr;

  return 1;
}

int str2val(char const* str,float & val, char const* & next) {
  char* error_ptr;
  double tmp_double;

  if (0 == str) {
    next= 0;
    return 0;
  }

  tmp_double= strtod(str, &error_ptr);

  if ( str == error_ptr
       || ERANGE == errno ) {
    next= error_ptr;
    return 0;
  }

  val= tmp_double;
  next= error_ptr;
  
  return 1;
}

int str2val(char const* str,bool & val, char const* & next) {
  bool tmp_bool;
  const char * dum;

  if (0 == str) {
    next= 0;
    return 0;
  }

  while ( IS_WHITE_CHAR(*str) )
    str++;
  
  if ( '1' == *str ) {
    tmp_bool= true;
    dum= str+1;
  }
  else if ( '0' == *str ) {
    tmp_bool= false;
    dum= str+1;
  }
  else if ( str[0] == 't' && str[1] == 'r' && str[2] == 'u' && str[3] == 'e') {
    tmp_bool= true;
    dum= str+4;
  }
  else if ( str[0] == 'f' && str[1] == 'a' && str[2] == 'l' && str[3] == 's' && str[4] == 'e') {
    tmp_bool= false;
    dum= str+5;
  }
  else {
    next= str;
    return 0;
  }

  val= tmp_bool;
  next= dum;
  
  return 1;
}

/*****************************************************************************/

int str2val(char const* str, int num,    int * val, char const* & next) {
  const char * dum;
  int size=0;
  if (num < 1) {
    next= str;
    return 0;
  }

  while (size < num) {
    int res= str2val(str, *val, dum);
    if (res) {
      val++;
      size++;
      str= dum;
    }
    else 
      break;
  }
  next= dum;
  return size;
}

int str2val(char const* str, int num,   long * val, char const* & next) {
  const char * dum;
  int size=0;
  if (num < 1) {
    next= str;
    return 0;
  }

  while (size < num) {
    int res= str2val(str, *val, dum);
    if (res) {
      val++;
      size++;
      str= dum;
    }
    else 
      break;
  }
  next= dum;
  return size;
}

int str2val(char const* str, int num, double * val, char const* & next) {
  const char * dum;
  int size=0;
  if (num < 1) {
    next= str;
    return 0;
  }

  while (size < num) {
    int res= str2val(str, *val, dum);
    if (res) {
      val++;
      size++;
      str= dum;
    }
    else 
      break;
  }
  next= dum;
  return size;
}

int str2val(char const* str, int num,  float * val, char const* & next) {
  const char * dum;
  int size=0;
  if (num < 1) {
    next= str;
    return 0;
  }

  while (size < num) {
    int res= str2val(str, *val, dum);
    if (res) {
      val++;
      size++;
      str= dum;
    }
    else 
      break;
  }
  next= dum;
  return size;
}

int str2val(char const* str, int num,   bool * val, char const* & next) {
  const char * dum;
  int size=0;
  if (num < 1) {
    next= str;
    return 0;
  }

  while (size < num) {
    int res= str2val(str, *val, dum);
    if (res) {
      val++;
      size++;
      str= dum;
    }
    else 
      break;
  }
  next= dum;
  return size;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
#if 0

#include <iostream>

int main(int argc, char **argv) {
  const char *str, *chr, *dum;
  str= "val             =         \t     3 4 \n 5";
  chr= str;

  if (strskip(chr,"val",dum)) {
    chr= dum;
    cout << "\nafter val [" << chr << "]";
    if (strskip(chr,"=",dum)) {
      chr= dum;
      cout << "\nafter = [" << chr << "]";
    }
  }
  else {
    cerr << "\n not valid args";
  }
    
  while (true) {
    int i= -1;
    if ( str2val(chr,i,chr) ) {
      cout << "\ngot " << i << flush;
      cout << "\nnext= [" << chr << "]" << flush;
    }
    else {
      cout << "\nend of string= [" << chr << "]";
      break;
    }
  }


  int arr[10];
  int res= str2val("T1   2  -1", 2, arr, str);
  cout << "\nres= " << res << ",";
  for (int i=0; i<res; i++)
    cout << " " << arr[i];
  cout << "\nrest= " << str;
  
  cout << endl;


  return 1;
}
#endif
