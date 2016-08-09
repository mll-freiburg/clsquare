#include "statecontroller.h"
#include "valueparser.h"
#include "global.h"
#include <fstream>
#include <sstream>

bool StateController::get_action(const double* observed_state, double* action)
{
  double resaction = 0.0;

  switch (mode) {
  case 0:
    if (!xwork.isWithinSet(observed_state, xdim)) return false;
    if (xplus.isWithinSet(observed_state, xdim)) cummulated_reward += 2.0;
    else cummulated_reward += 1.0;
    
    for (int i=0; i<xdim; i++) resaction+= (gain[i] * observed_state[i]);
    
    resaction+= offsetAction;
    
    if (resaction > maxAction) resaction = maxAction;
    if (resaction < minAction) resaction = minAction;

    action[0] = resaction;
    break;

  case 1:
    for (int i=0; i<udim; i++) {
      if ( ref_inp_fun[i] == 0)
        action[i]=0.0;
      else
        action[i] = ref_inp_fun[i]->get( steps * delta_t );
    }
    break;
  }
  
  steps++;
  return true;
}

bool StateController::init(const int observed_state_dim, const int action_dim, double deltat, const char* fname, const char* chapter)
{
  bool res = true;
  
  xdim    = observed_state_dim;
  udim    = action_dim;
  delta_t = deltat;
  mode    = 0; 
  steps   = 0;
  gain = new double[xdim];
  for (int i=0; i<xdim; i++) gain[i] = 0.0;
  maxAction =  1.0;
  minAction = -1.0;
  offsetAction = 0.0;

  ref_inp_fun.resize(udim);
  for (int i=0; i<udim; i++) ref_inp_fun[i]=0;
  
  if (chapter==0)
    res &= read_options(fname , "Controller" );
  else
    res &= read_options(fname , chapter );
  
  cummulated_reward = 0.0;
  trajctr = 0;

  return res;
}

void StateController::deinit()
{
  delete[] gain;
 
  //IOUT("We had " << trajctr << " trajectories, cummulated reward: " << cummulated_reward << " avrg " << cummulated_reward / trajctr << "\n");
  std::ofstream res("statecontroller.reward.res");
  res << cummulated_reward / trajctr << "\n";
  res.close();
}

void StateController::notify_episode_stops(const double* current_observed_state)
{
  steps=0;
  trajctr++;
}

bool StateController::read_options(const char* fname, const char* chapter)
{
  char paramstr[MAX_STR_LEN];
  ValueParser vp(fname,chapter);
  
  vp.get("maxAction", maxAction);
  vp.get("minAction", minAction);
  vp.get("offsetAction", offsetAction);

  vp.get("mode", mode);

  switch(mode) {
  case 0:
    vp.get("gain", gain , xdim);
    
    if (vp.get("xwork", paramstr, MAX_STR_LEN))
      {
        if (!xwork.parseStr(paramstr,xdim))
          {
            EOUT("Wrong param string xwork");
            return false;
          }
      }
    if (vp.get("xplus", paramstr, MAX_STR_LEN))
      {
        if (!xplus.parseStr(paramstr,xdim))
          {
            EOUT("Wrong param string xplus");
            return false;
          }
      }
    break;
  case 1:
    for (int i=0; i<udim; i++) {
      std::stringstream pname;
      pname << "ref_inp_fun_" << i;
      if (vp.get(pname.str().c_str(),paramstr,MAX_STR_LEN)>=0) {
        ref_inp_fun[i] = new InterpolLinFun1D();
        if (!ref_inp_fun[i]->load(paramstr)) {
          EOUT("Can not read file " << pname.str() << " as lin interpol fun");
          return false;
        }
      } else {
        EOUT("No param: " << pname.str() << " given, needed for this mode");
        return false;
      }
    }
    break;
  default:
    EOUT("Unknown control mode");
    return false;
    break;
  }
  
  return true;
}

REGISTER_CONTROLLER(StateController, "A state based controller with gains");
