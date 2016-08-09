#ifndef _MULTI_PLANT_H_
#define _MULTI_PLANT_H_

#include "plant.h"
#include <vector>

/** 
  * @ingroup PLANT */
class MultiPlant : public Plant {
 public:
  bool get_next_plant_state(const double *current_plant_state, const double *current_action, double *next_plant_state);
  bool get_measurement(const double *plant_state, double *measurement);
  bool init(int &plant_state_dim, int &measurement_dim, int &action_dim, double &delta_t, const char *fname=0, const char *chapter=0);
  bool check_initial_state(double *initial_plant_state);
  void deinit();
  void notify_episode_starts();
  void notify_episode_stops();
  void notify_command_string(const char* buf);
  void notify_suspend_for_aux_call_cmd();
  void notify_return_from_aux_call_cmd();
 protected:
  int _pdim, _i;
  std::vector<int> _pstart, _mstart, _astart;
  std::vector<Plant*> _plants;
};

#endif
