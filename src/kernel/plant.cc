
#include "plant.h"
#include "global.h"


/** 
    This file contains the default behavior for plant functions.
    The functions are arranged in hirachical order.
    Dependend on the need for external_signal and observations you can implement your function
    in the approprate level in your derived class.
    Note: Some functions have to be implemented at least at the lowest level to get a working plant.
**/


/** default next_stat behavior **/
bool Plant::get_next_plant_state(const double *current_plant_state, const double *current_action, const double *current_external_signal, double *next_plant_state){
  return get_next_plant_state(current_plant_state, current_action, next_plant_state);
}

bool Plant::get_next_plant_state(const double *current_plant_state, const double *current_action, double *next_plant_state){
  EOUT("Plant: No next_state function of plant was implemented. Please implement this function first.");
  return false;
}


/** default get_measurement  **/
bool Plant::get_measurement(const double *plant_state, const double *external_signal, double *measurement){
  return get_measurement(plant_state, measurement);
}

bool Plant::get_measurement(const double *plant_state, double *measurement){
  for (int i=0; i<__plant_state_dim; i++) {
    measurement[i]=plant_state[i];
  }
  return true;
}

/** default check_initial_state behavior **/
bool Plant::check_initial_state(double *initial_plant_state) 
{
  return true;
}

/** default init behaviour **/
bool Plant::init_main(int& plant_state_dim, int& measurement_dim, int& action_dim, int& external_signal_dim, double& delta_t, const char* fname, const char* chapter)
{
  bool res = init(plant_state_dim, measurement_dim, action_dim, external_signal_dim, delta_t, fname, chapter);
  __plant_state_dim =  plant_state_dim;
  __measurement_dim =  measurement_dim;
  return res;
}
bool Plant::init(int& plant_state_dim, int& measurement_dim, int& action_dim, int& external_signal_dim, double& delta_t, const char* fname, const char* chapter) {
  external_signal_dim = 0;
  return init(plant_state_dim, measurement_dim, action_dim, delta_t, fname);
}
bool Plant::init(int &plant_state_dim, int &measurement_dim, int &action_dim, double &delta_t, const char *fname, const char* chapter) {
  bool res = init(plant_state_dim, action_dim, delta_t, fname);
  measurement_dim = plant_state_dim;
  return res;
}
bool Plant::init(int &plant_state_dim, int &action_dim, double &delta_t, const char *fname, const char* chapter)
{
  EOUT("Plant: No init function of plant was implemented. Please implement an init function to set all dimensions proper. More errors will occur due to zero dimension of vectors ... ");
  return false;
}

void Plant::notify_command_string(const char* buf){
  IOUT("Warning: 'Got notified by pipe, but no notify command behaviour is implemented. Cmd string: "<<buf);
}
