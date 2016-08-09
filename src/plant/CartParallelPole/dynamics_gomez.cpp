#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <vector>
#include <deque>
#include <cmath>
#include "dynamics_gomez.h"

#define sgn(x)                  ((x >= 0) ? 1 : -1)

using namespace std;

//////////////////////////////////////////////////////////////////////
//
// Double Pole physics
//
//////////////////////////////////////////////////////////////////////

/*
static const double MUP         = 0.000002;
static const double GRAVITY     = -9.8;
*/

int WINDOW = 0;

int UNIFORM_NOISE = 0; 
#define GAUSSIAN_NOISE 0

void GomezMiikkulainen::next_state (const double* plant_state, const double* action, double* next_state)
{
  performAction(action);
  next_state[0] = _state[2];
  next_state[1] = _state[3];
  next_state[2] = _state[4];
  next_state[3] = _state[5];
  next_state[4] = _state[0];
  next_state[5] = _state[1];
}

//////////////////////////////////////////////////////////////////////

GomezMiikkulainen::GomezMiikkulainen ()
{  
  set_length(2.4, 0.5, 0.05);
  set_mass(1.0, 0.1, 0.01);
  set_double(true);
  set_delta(0.01);
  _MUP     = 0.000002;
  _GRAVITY = -9.8;
}

void GomezMiikkulainen::notify_episode_starts (const double* state)
{
  _dydx[0] = _dydx[1] = _dydx[2] = _dydx[3] =  _dydx[4] = _dydx[5] = 0.0;
  _state[1] = _state[3] = _state[5] = 0;
  _state[2] = state[0];
  _state[4] = state[2];
  _state[0] = state[4];
}  

#define one_over_256  0.0390625
void GomezMiikkulainen::step (double action, double *st, double *derivs)
{
  double force=action;
  double costheta_1,  costheta_2=0.0;
  double sintheta_1,  sintheta_2=0.0;
  double gsintheta_1, gsintheta_2=0.0; 
  double temp_1, temp_2=0.0;
  double ml_1, ml_2=0.0;
  double fi_1, fi_2=0.0;
  double mi_1, mi_2=0.0;
  
  if((force >= 0) && (force < one_over_256))
    force = one_over_256;
  if((force < 0) && (force > -one_over_256))
    force = -one_over_256;
  
  costheta_1 = cos(st[2]);
  sintheta_1 = sin(st[2]);
  gsintheta_1 = _GRAVITY * sintheta_1;
  ml_1 = _LENGTH_POLE1 * _MASS_POLE1;   
  temp_1 = _MUP * st[3] / ml_1;
  fi_1 = (ml_1 * st[3] * st[3] * sintheta_1) + (0.75 * _MASS_POLE1 * costheta_1 * (temp_1 + gsintheta_1));
  mi_1 = _MASS_POLE1 * (1 - (0.75 * costheta_1 * costheta_1));

  if(_DOUBLE){
    costheta_2 = cos(st[4]);
    sintheta_2 = sin(st[4]);
    gsintheta_2 = _GRAVITY * sintheta_2;
    ml_2 = _LENGTH_POLE2 * _MASS_POLE2;
    temp_2 = _MUP * st[5] / ml_2;
    fi_2 = (ml_2 * st[5] * st[5] * sintheta_2) + (0.75 * _MASS_POLE2 * costheta_2 * (temp_2 + gsintheta_2));
    mi_2 = _MASS_POLE2 * (1 - (0.75 * costheta_2 * costheta_2));
  }
  derivs[1] = (force + fi_1 + fi_2) / (mi_1 + mi_2 + _MASS_CART);
  derivs[3] = -0.75 * (derivs[1] * costheta_1 + gsintheta_1 + temp_1) / _LENGTH_POLE1;
  if(_DOUBLE)
    derivs[5] = -0.75 * (derivs[1] * costheta_2 + gsintheta_2 + temp_2) / _LENGTH_POLE2;
}

void GomezMiikkulainen::rk4 (double f, double* y, double* _dydx, double* yout)
{
	int i;
	double hh, h6, dym[6], dyt[6], yt[6];
	int vars = _DOUBLE ? 5 : 3;
       
	hh=_DELTA * 0.5;
	h6=_DELTA / 6.0;
	for (i=0; i<=vars; i++)
    yt[i] = y[i] + hh * _dydx[i];
	step(f, yt, dyt);
	dyt[0] = yt[1];
	dyt[2] = yt[3];
	dyt[4] = yt[5];
	for (i=0; i<=vars; i++)
    yt[i] = y[i] + hh * dyt[i];
	step(f, yt, dym);
	dym[0] = yt[1];
	dym[2] = yt[3];
	dym[4] = yt[5];
	for (i=0; i<=vars; i++) {
	  yt[i]=y[i] + _DELTA * dym[i];
	  dym[i] += dyt[i];
	}
	step(f, yt, dyt);
	dyt[0] = yt[1];
	dyt[2] = yt[3];
	dyt[4] = yt[5];
	for (i=0; i<=vars; i++)
	  yout[i] = y[i] + h6 * (_dydx[i] + dyt[i] + 2.0 * dym[i]);
}
	
void GomezMiikkulainen::performAction (const double *output)
{ 
  int i;
  double tmpState[6];
  double force = output[0];
  
  _dydx[0] = _state[1];
  _dydx[2] = _state[3];
  _dydx[4] = _state[5];
  step(force, _state, _dydx);
  rk4(force, _state, _dydx, _state);

  for (i=0; i<6; ++i)
    tmpState[i] = _state[i];

  _dydx[0] = _state[1];
  _dydx[2] = _state[3];
  _dydx[4] = _state[5];
  step(force, _state, _dydx);
  rk4(force, _state, _dydx, _state);
}

