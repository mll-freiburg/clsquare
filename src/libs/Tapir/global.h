#ifndef _TAPIR_GLOBAL_H_
#define _TAPIR_GLOBAL_H_

#include <iostream>

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
#define EOUT( __x__ ) ESTREAM << "\033[1;31m#ERROR\033[0m [Tapir]: " << __x__ << "\n" << std::flush
#endif

#ifndef IOUT 
#define IOUT( __x__ ) ISTREAM << "#INFO  [Tapir]: " << __x__ << "\n" << std::flush
#endif

#ifndef IOUTC
#define IOUTC( __v__ , __x__ ) {if (verbosity >= __v__) ISTREAM << "#INFO  [Tapir]: " << __x__ << "\n" << std::flush;}
#endif

#ifndef IOUTV
#define IOUTV( __vl__ , __v__ , __x__ ) {if (__vl__ >= __v__) ISTREAM << "#INFO  [Tapir]: " << __x__ << "\n" << std::flush;}
#endif

#ifndef WARNING_LEVEL
#define WARNING_LEVEL 10
#endif
#ifndef WOUT
#define WOUT( __l__ , __x__ ) {if (WARNING_LEVEL >= __l__) WSTREAM << "\033[1;33m#WARNING\033[0m [Tapir]: " << __x__ << "\n" << std::flush;}
#endif

#ifndef DEBUG
#define DOUT( __x__ )
#else
#define DOUT( __x__ ) DSTREAM << "#DEBUG [" << __PRETTY_FUNCTION__ << "]: " << __x__ << "\n" << std::flush
#endif

#endif
