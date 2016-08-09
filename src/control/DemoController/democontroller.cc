#include "democontroller.h"
#include "valueparser.h"

bool DemoController::get_action(const double* observed_state, double* action)
{
  action[0] = observed_state[0]* .5;
  if(verbosity)
    IOUT( "observed_state[0]: "<<observed_state[0]<<" -> action: "<<action[0]);
  if(action[0] > max_action){
    IOUT("max action exceed. Stop requested by get_action");
    return false;
  }
    
  return true;
}

bool DemoController::init(const int observation_dim, const int action_dim, double deltat, 
			  const char* fname, const char* chapter)
{
  bool res = true;
  verbosity = 1;

  standard_init(observation_dim, action_dim, deltat);  // realized in controller.h

  max_action = 1000;

  if (chapter==0)
    res &= read_options(fname , "Controller" );
  else
    res &= read_options(fname , chapter );

  if(verbosity)
    IOUT( "initialized ");
  return res;
}


void DemoController::notify_transition(const double* observed_state,
				       const double* action,
				       const double* next_observed_state,
				       const double reward, const bool is_terminal_state, 
				       const double terminal_reward){

  if(verbosity)
    IOUT( "notfied transition (only first state variable displayed): "
	  <<observed_state[0]<<" , "<<action[0]
	  <<" -> "<<next_observed_state[0]
	  <<" reward: "<<reward
	  <<" is_terminal_state: "<<is_terminal_state
	  <<" final_reward "<<terminal_reward);
}


void DemoController::notify_episode_starts(){
  if(verbosity)
    IOUT( "started ");
}

void DemoController::notify_episode_stops(const double* current_observed_state){
  if(verbosity)
    IOUT( "stopped in observed_state[0]: "<<current_observed_state[0]);
}

bool  DemoController::check_initial_state(const double* initial_observed_state, const int observation_dim){
  if(verbosity)
    IOUT( "initial state ok. ");
  return true;
}


bool DemoController::read_options(const char* fname, const char* chapter)
{
  ValueParser vp(fname,chapter);
  
  vp.get("max_action", max_action);
  vp.get("verbosity", verbosity);

  return true;
}

REGISTER_CONTROLLER(DemoController, "A simple controller for demo purposes.")
