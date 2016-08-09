#ifndef _FUNCGEN1D_H_
#define _FUNCGEN1D_H_

#include <vector>
#include <fstream>
#include <string>
#include <stdio.h>
#include "Socket.h"

/** General interface for a 1 dimensional function generator in the form of y=f(x).
  * This functions can depend on x (e.g. as a time index) but are not expected to depend
  * in this form also a user input can be seen as a function generator.
  * Can be used everywhere in code, especially important as reference input definition.
  */
class FuncGen1D {
    public:
    virtual ~FuncGen1D(){;}
    virtual bool    init(const char* config_fname, const char* config_chapter, const char* param_prefix ="")=0;
    virtual double  get( double x )=0;
};

class InterpolLinFun1D : public FuncGen1D {
 public:
  InterpolLinFun1D();
  virtual ~InterpolLinFun1D();
  
  virtual bool    init(const char* config_fname, const char* config_chapter, const char* param_prefix =""){return true;}
  virtual double  get( double x );
  bool            load(const std::string& fname);
  
 protected:
  // support points
  std::vector< double > xlist;
  std::vector< double > ylist;
  int lastid;  
};

class RefGUIClient : public FuncGen1D {
  public:
    RefGUIClient();
    virtual ~RefGUIClient();

    virtual bool   init(const char* config_fname, const char* config_chapter, const char* param_prefix =""){return true;}
    virtual double get( double x );

    bool   request_gui(std::string cmd);

  protected:
     TCPSocket* client;

};

#endif
