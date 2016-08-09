#ifndef _INTEGRATION_H_
#define _INTEGRATION_H_

namespace tools {

  class SysModel {
  public:
    SysModel(int _dim, double _delta_t, int _num_integration_steps, int _integration_mode, int _udim);
    SysModel();
    virtual ~SysModel();
    
    /** init function when default constructor is used **/
    void sysmodel_init(int _dim, double _delta_t, int _num_integration_steps, int _integration_mode, int _udim);
    
    /** System dgl for MDP */
    bool sysmodel_nextState(const double * state, const double * action, double * nextState);
    
    /** System dgl for non MDP dynamic systems */
    bool sysmodel_nextState(double t, const double * state, const double * action, double * nextState);
    
  protected:
    int     sysmodel_integration_mode;
    int     sysmodel_xdim;
    int     sysmodel_udim;
    int     sysmodel_num_integration_steps;
    double  sysmodel_delta_t;
    bool    sysmodel_inited;


    void rkdumb(const double * vstart, double t_start, double t_stop, int n, double * vend, const double *u);
    
    void rk4(const double * y, const double * dydx, double x, double interval, double * yout, const double *u);

    /** The derivative dgl defining the system. 
     * Implement your system here. An MDP should not depend on the variable t (time invariant). 
     */
    virtual void sysmodel_derivs(double t, const double *x, double *dxdt, const double *u)=0;

  };
}

#endif
