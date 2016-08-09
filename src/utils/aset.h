#ifndef _ASET_H_
#define _ASET_H_

#include <vector>
#include <iostream>

class Aset {
  
 protected:
  typedef std::vector<double> Aset_action_t;

 public:
  std::vector<Aset_action_t> action;


  Aset();

  bool getAction(int idx, float * u, int u_dim);
  bool getAction(int idx, double * u, int u_dim);
  bool parseStr(const char * str_to_parse, int u_dim);
  void print(std::ostream & out);
  int  size(){return (int)action.size();};

};

#endif
