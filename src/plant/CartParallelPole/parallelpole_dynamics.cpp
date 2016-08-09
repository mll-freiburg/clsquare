#include "parallelpole_dynamics.h"

void ParallelPoleDynamics::set_mass (const double mc, const double mp1, const double mp2)
{
  _MASS_POLE1 = mp1;
  _MASS_POLE2 = mp2;
  _MASS_CART  = mc;
}

void ParallelPoleDynamics::set_length (const double lt, const double lp1, const double lp2)
{
  _LENGTH_POLE1 = lp1;
  _LENGTH_POLE2 = lp2;
  _LENGTH_TRACK = lt;
}

void ParallelPoleDynamics::set_friction (const double fc, const double fp1, const double fp2)
{
  _FRIC_POLE1 = fp1;
  _FRIC_POLE2 = fp2;
  _FRIC_CART  = fc;
}

void ParallelPoleDynamics::set_double (bool dbl)
{
  _DOUBLE = dbl;
}

void ParallelPoleDynamics::set_delta (double delta)
{
  _DELTA = delta;
}
