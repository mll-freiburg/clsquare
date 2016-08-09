#include "standardcontroller.h"
#include "valueparser.h"
#include "global.h"
#include <fstream>
#include <sstream>

bool StandardController::get_action(const double* observed_state, double* action)
{
  double resaction = default_action;

  double e    = observed_state[error_id];
  double dedt = (e - this->last_err) / this->delta_t ;

  if (xwork.numSubsets()>0 &&  !xwork.isWithinSet(observed_state, xdim)) {
      // IOUT("ended with out of xwork.");
      return false;
  }

  switch (controller_mode) {
  case 0:
      break;
  case 2:
      resaction = ref_gui_client->get(episode_step_counter*delta_t);
      break;
  case 10: // PID control
      resaction = kp * e + kd * dedt;
      break;
  }
  
  if (resaction > max_action) resaction = max_action;
  if (resaction < min_action) resaction = min_action;

  action[0] = resaction;

  this->last_err = e;

  episode_step_counter++;
  return true;
}

bool StandardController::init(const int observation_dim, const int action_dim, double deltat, const char* fname, const char* chapter)
{
  bool res = true;
  
  xdim    = observation_dim;
  udim    = action_dim;
  delta_t = deltat;

  controller_mode       = 0;
  episode_counter       = 0;
  episode_step_counter  = 0;

  max_action            =  1.0;
  min_action            = -1.0;

  ref_gui_client        = 0;
  error_id              = 0;
  kp = ki = kd          = 0.0;

  if (chapter==0)
    res &= read_options(fname , "Controller" );
  else
    res &= read_options(fname , chapter );

  return res;
}

void StandardController::deinit()
{
    if (ref_gui_client != 0) delete ref_gui_client; ref_gui_client = 0;
}

void StandardController::notify_episode_stops(const double* current_observed_state)
{
  episode_step_counter = 0;
  episode_counter++;
}

bool StandardController::read_options(const char* fname, const char* chapter)
{
  char paramstr[MAX_STR_LEN];
  ValueParser vp(fname,chapter);
  
  if (vp.get("xwork", paramstr, MAX_STR_LEN)>0)
  {
      if (!xwork.parseStr(paramstr,xdim))
      {
          EOUT("Wrong param string xwork");
          return false;
      }
  }

  vp.get("max_action", max_action);
  vp.get("min_action", min_action);
  vp.get("default_action", default_action);

  vp.get("controller_mode", controller_mode);



  switch(controller_mode) {
  case 0:
      break;
  case 2:
      if (vp.get("gui_request_string",paramstr,MAX_STR_LEN)>=0) {
          ref_gui_client = new RefGUIClient();
          if (!ref_gui_client->request_gui( paramstr )) {
              EOUT("Can not create user input gui with request: " << paramstr);
              return false;
          }
      }
      break;
  case 10:
      vp.get("error_id" , error_id );
      vp.get("kp",kp);
      vp.get("ki",ki);
      vp.get("kd",kd);
      break;
  default:
      EOUT("Unknown controller_mode" << controller_mode);
      return false;
      break;
  }
  
  return true;
}

void StandardController::get_help( std::ostream& out )
{
    out
            << "xplus  \t \t SetDef \t the working range of the controller (plant_observation dim) \n"
            << "\t example: \"xwork = [0 1][0 10]\" for 2 dim observations of plant\n"
            << "max_action \t double \t the maximal action allowed\n"
            << "min_action \t double \t the minimal action allowed\n"
            << "default_action \t double \t the default action to be set\n"
            << "controller_mode \t int \t controller behaviour:\n"
            << "\t 0 : fixed action (default_action)\n"
            << "\t 2 : user input gui client\n"
            << "\t\t gui_request_string \t string \t request parameters for gui\n"
            << "\t\t\t example : gui_request_string = SLIDER Aktion -1 1 0 100 \n"
            << "\t 10 : PID controller\n"
            << "\t\t error_id \t int [0 odim-1] \t id of observation variable containing control deviation\n"
            << "\t\t kp, ki, kd \t double \t control coeffs\n"
            ;
}

REGISTER_CONTROLLER(StandardController, "A collection of standard control laws.")
