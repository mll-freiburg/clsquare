#include "sysmodel.h"
#include <iostream>
#include "global.h"

tools::SysModel::SysModel(int _xdim, double _delta_t, int _num_integration_steps, int _integration_mode, int _udim) 
  : sysmodel_integration_mode(_integration_mode), sysmodel_xdim(_xdim), sysmodel_udim(_udim), sysmodel_num_integration_steps(_num_integration_steps), 
  sysmodel_delta_t(_delta_t), sysmodel_inited(true)
{;}

tools::SysModel::SysModel()
  : sysmodel_integration_mode(-1), sysmodel_xdim(0), sysmodel_udim(0), sysmodel_num_integration_steps(0), 
  sysmodel_delta_t(0), sysmodel_inited(false)
{;}

tools::SysModel::~SysModel(){;}

void tools::SysModel::sysmodel_init(int _xdim, double _delta_t, int _num_integration_steps, int _integration_mode, int _udim)
{
  sysmodel_integration_mode 	= _integration_mode;
  sysmodel_xdim 		= _xdim;
  sysmodel_udim			= _udim;
  sysmodel_num_integration_steps = _num_integration_steps; 
  sysmodel_delta_t		= _delta_t;
  sysmodel_inited 		= true;
}

void tools::SysModel::rkdumb(const double * vstart, double t_start, double t_stop, int n,
			     double * vend, const double *u)
{
  double t,h;
  double v[sysmodel_xdim], vout[sysmodel_xdim], dv[sysmodel_xdim];
  
  //std::cout << "rk4: xstart: " << x_start << " xstop: " << x_stop << " n: " << n << "\n"; 

  for (int i=0; i<sysmodel_xdim; i++) {   // initialize
    v[i] = vstart[i];
  }

  t=t_start;
  h=(t_stop-t_start)/(double)n;
  for (int k=0; k<n; k++) {
    sysmodel_derivs(t,v,dv, u);
    rk4(v, dv, t, h, vout, u);
    if (t+h == t)
      IOUT ("WARNING: " << __PRETTY_FUNCTION__ << ": Step size too small!");
    t+=h;
    for (int i=0; i< sysmodel_xdim; i++) {
      v[i]=vout[i];
    }
  }
  for (int i=0; i< sysmodel_xdim; i++) {
    vend[i]=v[i];
  }
}


void tools::SysModel::rk4(const double * y, const double * dydx, double x, double interval,
			  double * yout, const double * u)
{
  double xh, hh, h6;
  double dym[sysmodel_xdim], dyt[sysmodel_xdim], yt[sysmodel_xdim];

  //std::cout << "rk4step with: y: " << y[0] << "  " << y[1] << " dydx: " << dydx[0] << "  " << dydx[1] << " x: " << x << " int: " << interval << " u: " << u[0] << "\n";

  hh=interval*0.5;
  h6=interval/6.0;
  xh=x+hh;
  for (int i=0; i<sysmodel_xdim; i++) yt[i]=y[i]+hh*dydx[i]; // First step

  sysmodel_derivs(xh,yt,dyt,u);                // second step
  for(int i=0; i<sysmodel_xdim; i++) yt[i]=y[i]+hh*dyt[i];

  sysmodel_derivs(xh, yt, dym, u);             // Third step
  for (int i=0; i<sysmodel_xdim; i++) {
    yt[i]=y[i]+interval*dym[i];
    dym[i] += dyt[i];
  }

  sysmodel_derivs(x+interval, yt, dyt, u);     //Fourth step
  
  for (int i=0; i<sysmodel_xdim; i++) {     // Accumulate increments with proper weights
    yout[i]=y[i]+h6*(dydx[i]+dyt[i]+2.0*dym[i]);
  }
}


bool tools::SysModel::sysmodel_nextState(const double * state, const double * action, double * nextState)
{
  switch (sysmodel_integration_mode) {
  case 1:
    rkdumb(state, 0, sysmodel_delta_t, sysmodel_num_integration_steps, nextState, action);
    break;
  default:
    EOUT("Inegration Mode : " << sysmodel_integration_mode << " unknown!");
    return false;
  }
  return true;
}

bool tools::SysModel::sysmodel_nextState(double t, const double * state, const double * action, double * nextState)
{
  switch (sysmodel_integration_mode) {
  case 1:
    rkdumb(state, t, t+sysmodel_delta_t, sysmodel_num_integration_steps, nextState, action);
    break;
  default:
    EOUT("Inegration Mode : " << sysmodel_integration_mode << " unknown!");
    return false;
  }
  return true;
}


#if 0
/*****************************************************
 * This is a test implementation. It implements a simple ordinal dgl that is 
 * analytical solved and solved with an approximation.
 */

class dglSystem : public tools::SysModel {
 protected:
  void derivs(double x, const double *y, double *dydx, const double *u);

 public:
  dglSystem();
};

void dglSystem::derivs(double x, const double *y, double *dydx, const double *u)
{
  dydx[0]=3*y[0]+x*x;
};

dglSystem::dglSystem() : SysModel(1, 0.01, 4, 1, 0)
{
  
};

double lsg_test(double x) {
  return -1.0/3.0 * x*x - 2.0/9.0 *x - 2.0/27.0 + 2.0/27.0 * exp(3.0 *x);
}

double lsg_test2(double x) {
  return x*x - 2*x + 2 - 2*exp(-x);
}

int main() {
  
  double x=0;
  double v[1]={0};

  dglSystem dgl;

  while (x<2) {
    double y = lsg_test(x);
    double v_new[1];

    dgl.nextState(x, v, 0, v_new);

    std::cout << x << "  " << y << "  " << v[0] << "\n";
    x+=0.01;
    v[0]=v_new[0];
  }

}
#endif
