#ifndef JOYSTICK_CONTROLLER_H_
#define JOYSTICK_CONTROLLER_H_
#define MAX_KEYS 10

#include "controller.h"
#include "Joystick.h"
#include "setdef.h"

#define MAX_CONTROLLER 5

/** Allows controlling a plant through a joystick or gamepad (MLLCartPole_real).
  * Designed to be used with the MLLCartPole_real plant.
  * Currently the joystick can control only the first
  * action dimension, using either axis 0 or buttons 6 and 7.
  * Which physical buttons these are depends on the model;
  * for the Thrustmaster Dual Trigger 3 in 1 that the
  * controller was developed for, it's the left analog
  * joystick and buttons L2 and R2.
  *
  * Configuration parameters:
  * \li \b jdev (int, default 0): number of the joystick handle \e /dev/input/js<jdev>
  * \li \b rdev (int, default 7): number of the rumble handle \e /dev/input/event<rdev>
  * \li \b xwork (SetDef): working area of the subcontroller
  * \li \b fail_controller_name (string): class name of the subcontroller
  * \li \b fail_controller_name (string): config chapter of the subcontroller
  *
  * @ingroup CONTROLLER
  * @ingroup STATIC
  * @author Roland Hafner */
class JoystickController : public Controller
{
public:
  virtual bool get_action (const double* state, double* action);
  virtual bool init (const int observed_state_dim, const int action_dim, double deltat, const char* fname=0, const char* chapter=0);
  virtual void deinit ();
  virtual void notify_episode_starts ();
  virtual void notify_episode_stops (const double* current_observed_state);

protected:
  Joystick*	joy;
  Rumble*   rumb;

  unsigned int udim;
  unsigned int odim;

  Controller*	fail_controller;
  std::string	fail_controller_name;
  std::string	fail_controller_fname;
  bool  fail_controller_active;

  Controller*	aux_controller[MAX_CONTROLLER];
  std::string	aux_controller_name[MAX_CONTROLLER];
  std::string	aux_controller_fname[MAX_CONTROLLER];
  bool aux_controller_active[MAX_CONTROLLER];
  int  aux_controller_button[MAX_CONTROLLER];

  std::string joydev;
  std::string rumbdev;
  SetDef xwork;

  bool read_options(const char* fname, const char* chapter);
};

#endif
