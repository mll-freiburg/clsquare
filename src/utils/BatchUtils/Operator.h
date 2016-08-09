#ifndef _operator_h_
#define _operator_h_

namespace BatchUtils {

  template<class T> class Operator
  {
  public:
    virtual void operator() (T& arg)=0;
  };
}

#endif