
#include "valueparser.h"
#include "joystickcontroller.h"
#include "global.h"
#include <stdio.h>
#include <cmath>

/**********************************************************************/
/* procedures depending on function approximator type                 */
/**********************************************************************/

bool JoystickController::get_action(const double* observed_state, double* action)
{
  const std::vector< double >& axis = joy->get_axis_state();
  const std::vector< bool   >& button = joy->get_button_state();

  bool set = false;

  // aux controller 1
  for (int i=0; i<MAX_CONTROLLER; i++) {
    if (!set && aux_controller[i] && button[aux_controller_button[i]]) {
      if (!aux_controller_active[i]) {
        aux_controller[i]->notify_episode_starts();
        aux_controller_active[i] = true;
      }
      double tmp_state[odim];
      for (int h=0; h<(int)odim; h++) tmp_state[h]=observed_state[h];
      if (i==1) {
        tmp_state[0] += M_PI;  // turn 0-pos to bottom for suspension HACK
        while (tmp_state[0] >  M_PI ) tmp_state[0] -= 2 * M_PI;
      }
      if (aux_controller[i]->get_action(tmp_state , action)) {
        set = true;
      } else {
        aux_controller[i]->notify_episode_stops(observed_state);
        aux_controller_active[i] = false;
      }
    } else if (aux_controller_active[i]) {
      aux_controller[i]->notify_episode_stops(observed_state);
      aux_controller_active[i] = false;
    }
  }

  // fail controller
  if (!set && !xwork.isWithinSet(observed_state, odim)) {
    rumb->play(0);
    if (fail_controller) {
      if (!fail_controller_active) {
        fail_controller->notify_episode_starts();
        fail_controller_active = true;
      }
      if (!fail_controller->get_action(observed_state , action)) return false;
      set = true;
    } else
      return false;
  } else if (fail_controller_active) {
    fail_controller->notify_episode_stops(observed_state);
    fail_controller_active = false;
  }

  if (!set) {

    if (button[6]) {
      action[0]=-1.0;
      return true;
    }
    if (button[7]) {
      action[0]= 1.0;
      return true;
    }

    // joystick actions
    action[0] = axis[0];
    return true;
  }

  return true;
}

bool JoystickController::init(const int observed_state_dim, const int action_dim, double deltat, const char *fname, const char* chapter)
{

  joydev		= "/dev/input/js1";
  rumbdev		= "/dev/input/event7";
  udim			= action_dim;
  odim			= observed_state_dim;

  xwork			= SetDef();

  fail_controller	= 0;
  fail_controller_name	= "";
  fail_controller_fname = fname;
  fail_controller_active = false;

  for (int i=0; i<MAX_CONTROLLER; i++) {
    aux_controller[i]		= 0;
    aux_controller_name[i]	= "";
    aux_controller_fname[i]	= fname;
    aux_controller_button[i]	= i;
    aux_controller_active[i]	= false;
  }

  if (!read_options(fname, chapter==0 ? "Controller" : chapter)) {
    EOUT("in reading options.");
    return false;
  }

  if (fail_controller_name!="") {
    fail_controller = ControllerFactory::getTheControllerFactory()->create(fail_controller_name.c_str());
    if (fail_controller == 0) {
      EOUT ("No implementation found for Controller (" <<  fail_controller_name << ")!");
      return false;
    }
    if (!fail_controller->init(observed_state_dim,
                               action_dim,
                               deltat,
                               fail_controller_fname.c_str())) {
      EOUT("Can not init fail_controller");
      return false;
    }
  }

  for (int i=0; i<MAX_CONTROLLER; i++) {
    if (aux_controller_name[i]!="") {
      aux_controller[i] = ControllerFactory::getTheControllerFactory()->create(aux_controller_name[i].c_str());
      if (aux_controller[i] == 0) {
        EOUT ("No implementation found for Controller (" <<  aux_controller_name[i] << ")!");
        return false;
      }
      if (!aux_controller[i]->init(observed_state_dim,
                                   action_dim,
                                   deltat,
                                   aux_controller_fname[i].c_str())) {
        EOUT("Can not init aux_controller" << i+1);
        return false;
      }
      IOUT("Inited aux_controller" << i+1 << " " << aux_controller_name[i]);
    }
  }

  joy = 0;
  try {
    joy = new Joystick(joydev.c_str());
  } catch (std::exception& e) {
    joy = 0;
    EOUT("No Joystick available ... on " << joydev << " : " << e.what());
    return false;
  }

  rumb = new Rumble(rumbdev.c_str());

  return true;
}

bool JoystickController::read_options(const char* fname, const char* chapter)
{
  ValueParser vp(fname,chapter);
  char paramstr[1024];
  int nread;

  if ((nread=vp.get("xwork", paramstr, 1024)) > 0) {
    if (!xwork.parseStr(paramstr , odim)) {
      EOUT("in parsing xwork!");
      return false;
    }
  }

  int jd=0;
  char jdbuf[200];
  vp.get("jdev" , jd);
  sprintf(jdbuf,"/dev/input/js%d",jd);
  joydev = jdbuf;

  vp.get("rdev" , jd);
  sprintf(jdbuf,"/dev/input/event%d",jd);
  rumbdev = jdbuf;

  if ((nread=vp.get("fail_controller_name", paramstr, 1024)) > 0) {
    fail_controller_name = paramstr;
  }
  if ((nread=vp.get("fail_controller_fname", paramstr, 1024)) > 0) {
    fail_controller_fname = paramstr;
  }

  for (int i=0; i<MAX_CONTROLLER; i++) {
    char strbuf[1024];

    sprintf(strbuf , "aux_controller_name%d" , i+1);
    if ((nread=vp.get(strbuf, paramstr, 1024)) > 0) {
      aux_controller_name[i] = paramstr;
    }
    sprintf(strbuf , "aux_controller_fname%d" , i+1);
    if ((nread=vp.get(strbuf, paramstr, 1024)) > 0) {
      aux_controller_fname[i] = paramstr;
    }
  }

  return true;
}

void JoystickController::notify_episode_starts()
{
}

void JoystickController::notify_episode_stops (const double* current_observed_state)
{
  if (fail_controller && fail_controller_active) fail_controller->notify_episode_stops(current_observed_state);
  fail_controller_active = false;
  for (int i=0; i<MAX_CONTROLLER; i++) {
    if (aux_controller[i] && aux_controller_active[i]) aux_controller[i]->notify_episode_stops(current_observed_state);
    aux_controller_active[i] = false;
  }
}

void JoystickController::deinit ()
{
  if (joy !=0) delete joy;
  joy=0;
  if (fail_controller) fail_controller->deinit();

  for (int i=0; i<MAX_CONTROLLER; i++) {
    if (aux_controller[i]) aux_controller[i]->deinit();
  }
}

REGISTER_CONTROLLER(JoystickController,"Control with Joystick.")

