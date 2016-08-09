#ifndef _batchdata_h_
#define _batchdata_h_

#include<vector>
#include<iostream>
#include<exception>

#include "VectorArithmetic.h"

namespace BatchUtils {

  using std::vector;
  using std::ostream;
  using std::istream;
  using std::exception;
  
  /** A class that represents a single state of the state space. Can be
   constructed by either specifying an STL-vector of doubles or a standard
   C array of doubles. */
  class X {
  public:
    vector<double> x;          ///< the state
    
    /** constructs a zero-initialized state of the given dimension */
    X(int dim=0) : x(dim) {}
    /** constructs a state by copying the values from the given double 
     vector */
    X(const vector<double>& x) : x(x) {}
    /** constrcuts a state by copying the values from the given double
     array */
    X(const double* x, int dim) : x(&x[0], (&x[0])+dim) {}
    /** destructor */
    virtual ~X() {}
  };

  /** This class represents a pair of a state and its state-value as for
   example can be used to represent samples of a state-value function. Since
   this class is used during batch learning to store the target values of a
   state-value function, the second element of the pair is named "target". */ 
  class XT : public X {
  public:
    double target;       ///< the target value of the state X            
    
    /** constructs a zero-initialized state and state-value pair of the 
     specified dimension */
    XT(int dim=0) : X(dim), target(0) {} 
    /** constructs a stateu and state-value pair of the given arguments */
    XT(const vector<double>& x, double target) : X(x), target(target) {}
    /** constructs a stateu and state-value pair of the given arguments */
    XT(const double* x, double target, int dim) : X(x, dim), target(target) {}
    virtual ~XT() {}
  };

  /** This class represents a state-action pair and the corresponding target
   value as determined in batch Reinforcement Learning algorithms. */
  class XUT : public XT {
  public:
    vector<double> u;    ///< the action of the state-action pair
    
    /** constructs a zero-initialized state-action pair and target value */
    XUT(int xdim=0, int udim=0) : XT(xdim), u(udim) {}
    XUT(const vector<double>& x, const vector<double>& u, double target) : XT(x, target), u(u) {}
    XUT(const double* x, const double* u, double target, int xdim, int udim) : XT(x, target, xdim), u(&u[0], (&u[0])+udim) {}
    virtual ~XUT() {}
  };
  
  /** This class represents a state-action pair and the corresponding target
   vlaue as determined in batch Reinforcement Learning algorithms. The action
   is represented by an index referring to a specific action in the finite 
   set of (multi-dimensional) actions */
  class XUiT : public XT {
  public:
    int ui;
    
    /** constructs a zero-initialized state-action pair and target value */
    XUiT(int xdim=0) : XT(xdim), ui(0) {}
    XUiT(const vector<double>& x, int ui, double target) : XT(x, target), ui(ui) {}
    XUiT(const double* x, int ui, double target, int xdim) : XT(x, target, xdim), ui(ui) {}
    XUiT(const XUT& xux, const vector<vector<double> >& all_actions);
    virtual ~XUiT() {}
  };

  class XUiX;  // forward declaration, needed as return type

  /** This class represents a transition from a state to a subsequent state
   using an action u */
  class XUX : public X {
  public:
    vector<double> u;      ///< the action used in this transition
    vector<double> xnext;  ///< the subsequent state in the transition
    double r;              ///< the reward observed after executing action u in the state x
    bool isTerminal;       ///< a flag indicating whether or not this transition was to a terminal state
    
    /** constructs a transition with zero-initialized states, action and costs */ 
    XUX(int xdim=0, int udim=0) : X(xdim), u(udim), xnext(xdim), r(0.), isTerminal(false) {}
    /** constructs a transition by copying the values from the given arguments */ 
    XUX(const vector<double>& x, const vector<double>& u, const vector<double>& xnext, double r, bool isTerminal=false) : X(x), u(u), xnext(xnext), r(r), isTerminal(isTerminal) {}
    /** constructs a transition by copying the values from the given arguments */ 
    XUX(const double* x, const double* u, const double* xnext, double r, bool isTerminal, int xdim, int udim) : X(x, xdim), u(&u[0], (&u[0])+udim), xnext(&xnext[0], (&xnext[0])+xdim), r(r), isTerminal(isTerminal) {}
    
    /** converts a transition of the XUX-representation to a transition 
     using the index of the action in the specified set of actions
     all_actions */
    const XUiX toXUiX(const vector<vector<double> >& all_actions) const;
  };

  /** This class represents a transition from a state to a subsequent state
   using an action u, where the action is represented by an index referring
   to a specific (multi-dimensional) action in the finite set of actions. */
  class XUiX : public X {
  public:
    int ui;                ///< the index of the action used in this transition
    vector<double> xnext;  ///< the subsequent state in this transition
    double r;              ///< the reward observed after executing action ui in the state x
    bool isTerminal;       ///< a flag indicating whether or not this transition was to a terminal state
    
    /** constructs a transition with zero-initialized states, action and costs */ 
    XUiX(int xdim=0) : X(xdim), ui(0), xnext(xdim), isTerminal(false) {}
    /** constructs a transition by copying the values from the given arguments */ 
    XUiX(const vector<double>& x, int ui, const vector<double>& xnext, double r, bool isTerminal=false) : X(x), ui(ui), xnext(xnext), r(r), isTerminal(isTerminal) {}
    /** constructs a transition by copying the values from the given arguments */ 
    XUiX(const double* x, int ui, const double* xnext, double r, bool isTerminal, int xdim) : X(x, xdim), ui(ui), xnext(&xnext[0], (&xnext[0])+xdim), r(r), isTerminal(isTerminal) {}  
    
    /** converts the given transition xux to the representation using the
     action's index by looking up the index in the set of all_actions */ 
    XUiX(const XUX& xux, const vector<vector<double> >& all_actions);
    /** constructs a transition by copying the values from the given arguments 
     and looking up the index of the given action u in the set of 
     all_actions */ 
    XUiX(const vector<double>& x, const vector<double>& u, const vector<double>& xnext, double r, bool isTerminal, const vector<vector<double> >& all_actions);
    
    /** converts a transtion of the XUiX-representation to a transition
     using the values of the action. */
    const XUX toXUX(const vector<vector<double> >& all_actions) const;
  };

  /** This class stores both representations of transitions --- using the
   indecees of actions or using the particular values of (multi-dimensional)
   actions --- in parallel. */
  class XUXDual : public XUX 
  {
  public:
    int ui;         ///< the index of the action used in this transition
    
    XUXDual(int xdim=0, int udim=0) : XUX(xdim, udim), ui(0) {}
    XUXDual(const vector<double>& x, const vector<double>& u, const vector<double>& xnext, double r, bool isTerminal, const vector<vector<double> >& all_actions);
    XUXDual(const vector<double>& x, int ui, const vector<double>& xnext, double r, bool isTerminal, const vector<vector<double> >& all_actions);
    XUXDual(const XUX& xux, const XUiX& xuix) : XUX(xux), ui(xuix.ui) {}
  };
  

  /** This class is used internally in order to allow more comfortable
   handling ("auto-casting") of vectors the batch-data types. */
  template<class T>
  class XVectorWrapper
  {
  public:
    XVectorWrapper(const vector<T>& data) : data(data) {}
    const X& operator[] (int i) const { return data[i]; }
    int size() const { return data.size(); }
    
  protected:
    const vector<T>& data;
  };
  
  /** This class is used internally in order to allow more comfortable
   handling ("auto-casting") of vectors the batch-data types. */
  template<class TSRC, class TTRG>
  class CastingVectorWrapper
  {
  public:
    CastingVectorWrapper(const vector<TSRC>& data) 
    : data(data) 
    {
      TSRC test;
      if (dynamic_cast<TTRG*> (&test) == 0) {  // check, that TTRG is a valid base class of TSRC
        throw(exception());
      }
    }
    const TTRG& operator[] (int i) const { return data[i]; }       // casts an element of the derived class TSRC to the parent-class TTRG using polymorphism
    int size() const { return data.size(); }
    
  protected:
    const vector<TSRC>& data;
  };
  
  
  /** This class represents a transition from a state to a subsequent state
   using an action u, where the states and the action are represented by 
   indecees referring to entries in fixed lists of states and actions. 
   This representation may be used to speed-up batch learning. */  
  class XiUiXi {
  public:
    int x;
    int u;
    int xnext;
    double r;
    bool isTerminal;
    
    XiUiXi(int x, int u, int xnext, double r, bool isTerminal=false) : x(x), u(u), xnext(xnext), r(r), isTerminal(isTerminal) {}
  };
  
  
  /** This class represents a state-action pair and target value
   where the state and the action are represented by 
   indecees referring to entries in fixed lists of states and actions. 
   This representation may be used to speed-up batch learning. */    
  class XiUiT {
  public:
    int x;
    int u;
    double target;
    
    XiUiT() : x(0), u(0), target(0) {}
    XiUiT(int x, int u, double target) : x(x), u(u), target(target) {}
  };
  
  /** translates the transitions specified in the vector src to the
   representation using indecees of actions and stores the resulting
   representations in the trg vector */
  void translate(const vector<XUX>& src, vector<XUiX>& trg, const vector<vector<double> >& actions);
  /** translates the state-action pairs and target values specified 
   in the vector src to the representation using indecees of actions 
   and stores the resulting representations in the trg vector */  
  void translate(const vector<XUT>& src, vector<XUiT>& trg, const vector<vector<double> >& actions);

}  
  
// inlines ///////

namespace BatchUtils {
  
  inline std::istream& operator>>(std::istream& in, BatchUtils::XUiX& xuix)
  {
    in >> xuix.x >> xuix.ui >> xuix.xnext >> xuix.r >> xuix.isTerminal;
    return in;
  }
  inline std::ostream& operator<<(std::ostream& out, const BatchUtils::XUiX& xuix)
  {
    out << xuix.x << " " << xuix.ui << " " << xuix.xnext << " " << xuix.r << " " << xuix.isTerminal; 
    return out;
  }
  

inline std::istream& operator>>(std::istream& in, BatchUtils::XUX& xux)
{
  in >> xux.x >> xux.u >> xux.xnext >> xux.r >> xux.isTerminal;
  return in;
}
inline std::ostream& operator<<(std::ostream& out, const BatchUtils::XUX& xux)
{
  out << xux.x << " " << xux.u << " " << xux.xnext << " " << xux.r << " " <<  xux.isTerminal;
  return out;
}
  
  inline std::istream& operator>>(std::istream& in, BatchUtils::XUiT& xut)
  {
    for (unsigned int i=0; i < xut.x.size(); i++) {
      in >> xut.x[i];
    }
    in >> xut.ui;
    in >> xut.target;
    return in;
  }
  inline std::ostream& operator<<(std::ostream& out, const BatchUtils::XUiT& xut)
  {
    for (unsigned int i=0; i < xut.x.size(); i++) {
      out << xut.x[i] << " ";
    }
    out << xut.ui << " ";
    out << xut.target;
    return out;
  }

inline std::istream& operator>>(std::istream& in, BatchUtils::XUT& xut)
{
  for (unsigned int i=0; i < xut.x.size(); i++) {
    in >> xut.x[i];
  }
  for (unsigned int i=0; i < xut.u.size(); i++) {
    in >> xut.u[i];
  }
  in >> xut.target;
  return in;
}
inline std::ostream& operator<<(std::ostream& out, const BatchUtils::XUT& xut)
{
  for (unsigned int i=0; i < xut.x.size(); i++) {
    out << xut.x[i] << " ";
  }
  for (unsigned int i=0; i < xut.u.size(); i++) {
    out << xut.u[i] << " ";
  }
  out << xut.target;
  return out;
}

inline std::istream& operator>>(std::istream& in, BatchUtils::XT& xt)
{
  for (unsigned int i=0; i < xt.x.size(); i++) {
    in >> xt.x[i];
  }
  in >> xt.target;
  return in;
}
inline std::ostream& operator<<(std::ostream& out, const BatchUtils::XT& xt)
{
  for (unsigned int i=0; i < xt.x.size(); i++) {
    out << xt.x[i] << " ";
  }
  out << xt.target;
  return out;
}

inline std::istream& operator>>(std::istream& in, BatchUtils::X& x)
{
  for (unsigned int i=0; i < x.x.size(); i++) {
    in >> x.x[i];
  }
  return in;
}
inline std::ostream& operator<<(std::ostream& out, const BatchUtils::X& x)
{
  for (unsigned int i=0; i < x.x.size(); i++) {
    out << x.x[i] << (i < x.x.size()-1 ? " " : "");
  }
  return out;
}
}

#endif