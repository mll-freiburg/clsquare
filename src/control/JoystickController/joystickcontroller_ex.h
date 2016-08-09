#ifndef JOYSTICK_CONTROLLER_H_
#define JOYSTICK_CONTROLLER_H_

#include "controller.h"
#include "Joystick.h"

/** Allows controlling a plant through a joystick or gamepad (plant independent).
  * More general version of the JoystickController module.
  * Auxiliary controller features have been removed, as they can be implemented
  * in a more versatile manner using the MixControl module.
  *
  * The binding of the joystick and the rumble pack can be adjusted through
  * the parameters \b jdev and \b rdev, with the defaults being
  * \e /dev/input/js0 and \e /dev/input/event7, respectively.
  *
  * Keymapping for Thrustmaster gamepad:
  *
  * Axes:
  * \li 0: left joystick, horizontal
  * \li 1: left joystick, vertical
  * \li 2: right joystick, horizontal
  * \li 3: left trigger (red mode), unused (green mode)
  * \li 4: right trigger (red mode), unused (green mode)
  * \li 5: right joystick, vertical, trigger difference (green mode)
  * \li 6: crosspad, horizontal
  * \li 7: crosspad, vertical
  *
  * Buttons:
  * \li 8:  square (1)
  * \li 9:  cross (2)
  * \li 10: circle (3)
  * \li 11: triangle (4)
  * \li 12: L1 (5)
  * \li 13: R1 (6)
  * \li 14: L2 (7)
  * \li 15: R2 (8)
  * \li 16: select (9)
  * \li 17: start (10)
  * \li 18: unused
  * \li 19: unused
  *
  * @ingroup CONTROLLER
  * @ingroup STATIC
  * @author Thomas Lampe */
class JoystickControllerEX : public Controller
{
public:
  virtual bool get_action (const double* state, double* action);
  virtual bool init (const int observed_state_dim, const int action_dim, double deltat, const char* fname=0, const char* chapter=0);
  virtual void deinit ();

protected:
  Joystick*	_joy;
  Rumble*   _rumb;

  unsigned int _udim, _adim, _bdim, _i;
};

#endif
