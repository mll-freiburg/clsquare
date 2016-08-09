/*! \file CartPole.h
 */

#ifndef _CARTPOLE_H_
#define _CARTPOLE_H_

#include <vector>
#include <deque>
#include "parallelpole_dynamics.h"

//! Pole Balancing domain
/*! 
    Implements the pole balancing dynamics using
    the Runge-Kutta 4th-order integration method.
    Can be instantiated with one or two poles.
*/
class GomezMiikkulainen : public ParallelPoleDynamics {

public:
  GomezMiikkulainen ();
  virtual ~GomezMiikkulainen () {};
  void next_state (const double* state, const double* action, double* next_state);
  void notify_episode_starts (const double* state);

protected:
  void performAction (const double *output);
  void step(double action, double *state, double *derivs);
  void rk4(double f, double y[], double dydx[], double yout[]);

  double _state[6];
  double _dydx[6];
  double _longPoleAngle;

  double _MUP;
  double _GRAVITY;
};

#endif

