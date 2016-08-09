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
   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
   DAMAGE. 
   
*/

/** Global definitions for CLSquare. */
  
#ifndef _CLSQUARE_GLOBAL_H_
#define _CLSQUARE_GLOBAL_H_

#include <iostream>
#include <exception>
#include <string>
#include <sstream>

// --- SOME USEFUL DEFINES ---
#define MAX_STR_LEN 2000

/*! \def XCODE
  * A central define for enabling Xcode features in the code.
  * If commented out the features will not be active.*/
// #define XCODE


// --- MESSAGE MACROS ----

#ifndef ESTREAM
#define ESTREAM std::cerr
#endif

#ifndef ISTREAM
#define ISTREAM std::cout
#endif

#ifndef WSTREAM
#define WSTREAM std::cerr
#endif

#ifndef DSTREAM
#define DSTREAM std::cerr
#endif

#ifndef EOUT
#define EOUT( __x__ ) ESTREAM << "\033[1;31m#ERROR\033[0m [" << __PRETTY_FUNCTION__ << "]: " << __x__ << "\n" << std::flush
#endif

#ifndef IOUT 
#define IOUT( __x__ ) ISTREAM << "#INFO  [" << __PRETTY_FUNCTION__ << "]: " << __x__ << "\n" << std::flush
#endif

#ifndef IOUTC
#define IOUTC( __v__ , __x__ ) {if (verbosity >= __v__) ISTREAM << "#INFO  [" << __PRETTY_FUNCTION__ << "]: " << __x__ << "\n" << std::flush;}
#endif

#ifndef IOUTV
#define IOUTV( __vl__ , __v__ , __x__ ) {if (__vl__ >= __v__) ISTREAM << "#INFO  [" << __PRETTY_FUNCTION__ << "]: " << __x__ << "\n" << std::flush;}
#endif

#ifndef WARNING_LEVEL
#define WARNING_LEVEL 10
#endif
#ifndef WOUT
#define WOUT( __l__ , __x__ ) {if (WARNING_LEVEL >= __l__) WSTREAM << "\033[1;33m#WARNING\033[0m [" << __PRETTY_FUNCTION__ << "]: " << __x__ << "\n" << std::flush;}
#endif

#ifndef DEBUG
#define DOUT( __x__ )
#else
#define DOUT( __x__ ) DSTREAM << "#DEBUG [" << __PRETTY_FUNCTION__ << "]: " << __x__ << "\n" << std::flush
#endif

// ---- CENTRAL EXCEPTION DEFINTION/MACRO ----

/** This is the central exception a code fragment can throw if an internal error occurs: Don't use this in init routines.
  * Throwing this exception in the loop cycle will terminate all sequences and is ment for non recoverable errors.
  * If critical is false the program will quit and report the message but will not name it an error.
  * This can be used to quit the program from inside the code.
  */
class CLSquareException: public std::exception
{
public:
    CLSquareException(std::string _desc , bool _critical=true)
            : desc(_desc), critical(_critical) {;}
    virtual ~CLSquareException() throw() {;}
    virtual const char* what() const throw()
    { return desc.c_str();}
    virtual bool is_critical() { return critical; }

protected:
    std::string desc;
    bool critical;
};

/** Central define for an unrecoverable error that will break the program,
  * but will deinit all modules properly (take this instead of an exit call).**/
#define CLSERR( __x__ ) {std::stringstream ss; ss << "FILE: " << __FILE__ << " LINE: " << __LINE__ << " FUNC: " << __PRETTY_FUNCTION__ << "\nMessage:\n" << __x__; throw CLSquareException(ss.str());}
#define CLSQUIT( __x__ ) {std::stringstream ss; ss << "Quit Request from: " << __PRETTY_FUNCTION__ << "\nMessage:\n" << __x__; throw CLSquareException(ss.str() , false );}

#endif
