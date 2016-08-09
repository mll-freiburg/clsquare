#include "valueparser.h"
#include "joystickcontroller_ex.h"
#include "global.h"
#include <stdio.h>
#include <cmath>

bool JoystickControllerEX::get_action (const double* observed_state, double* action)
{
  const std::vector<double>& axis = _joy->get_axis_state();
  const std::vector<bool>& button = _joy->get_button_state();
  for (_i=0; _i<_adim; _i++)
    action[_i] = axis[_i];
  for (_i=0; _i<_bdim; _i++)
    action[_i+_adim] = button[_i];
  for (_i=_adim+_bdim; _i<_udim; _i++)
    action[_i] = 0.;
  return true;
}

bool JoystickControllerEX::init (const int observed_state_dim, const int action_dim, double deltat, const char *fname, const char* chapter)
{
  ValueParser vp(fname, chapter==0 ? "Controller" : chapter);
  char dev[100];

  // connect joystick
  if (vp.get("jdev", dev, 100) < 1)
    sprintf(dev, "/dev/input/js0");
  try {
    _joy = 0;
    _joy = new Joystick(dev);
  } catch (std::exception& e) {
    EOUT("No Joystick available on " << dev << " : " << e.what());
    return false;
  }

  // connect rumble
  if (vp.get("rdev", dev, 100) < 1)
    sprintf(dev, "/dev/input/event7");
  _rumb = new Rumble(dev);

  // check dimensions
  _udim     = action_dim;
  _adim = _joy->get_axis_state().size();
  _bdim = _joy->get_button_state().size();
  unsigned jdim = _adim + _bdim;
  if (jdim < _udim)
    WOUT(10, "Joystick does not provide enough events to fill action vector (" << _adim << " axes + " << _bdim << " buttons vs. " << _udim << " actions). Remaining entries will be left blank.");
  if (jdim > _udim)
    WOUT(10, "Joystick event number exceeds action dimensionality (" << _adim << " axes + " << _bdim << " buttons vs. " << _udim << " actions). Tailing events will be ignored.");
  if (_adim > _udim) _adim = _udim;
  if ( jdim > _udim) _bdim = _udim - _adim;

  return true;
}

void JoystickControllerEX::deinit ()
{
  if (_joy != 0)
    delete _joy;
  _joy = 0;
}

REGISTER_CONTROLLER(JoystickControllerEX, "Control with Joystick.")
