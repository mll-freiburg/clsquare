#include "multiplant.h"
#include "valueparser.h"
#include <iostream>
#include <cstring>

#define SSSET(xxx) { ss.str(""); ss << xxx; };

bool MultiPlant::get_next_plant_state(const double *state, const double *action, double *next)
{
  for (_i=0; _i<_pdim; _i++)
{
    if (!_plants[_i]->get_next_plant_state(&state[_pstart[_i]], &action[_astart[_i]], &next[_pstart[_i]]))
      return false;
}
  return true;
}

bool MultiPlant::get_measurement(const double *plant_state, double *measurement)
{
  for (_i=0; _i<_pdim; _i++)
    if (!_plants[_i]->get_measurement(&plant_state[_pstart[_i]], &measurement[_mstart[_i]]))
      return false;
  return true;
}

bool MultiPlant::init(int &plant_state_dim, int &measurement_dim, int &action_dim, double &delta_t, const char *fname, const char *chapter)
{
  if (chapter==NULL) chapter = "Plant";
  ValueParser vp(fname, chapter==NULL ? "Controller" : chapter);
  std::stringstream ss;

  plant_state_dim = measurement_dim = action_dim = 0;
  for (_i=0; ; _i++) {

    // try to get next controller name
    int len, space = -1;
    char name[255];
    SSSET("plant_" << _i);
    len = vp.get(ss.str().c_str(), name, 255);
    if (len < 0) break;

    // if there's a space, treat next segment as chapter
    for (int _k=0; _k<len-1; _k++)
      if (name[_k] == ' ') {
        space = _k;
        name[_k] = '\0';
    }

    // create controller
    Plant *base = PlantFactory::getThePlantFactory()->create(name);
    if (base == NULL) {
      EOUT("MultiPlant could not create a plant of type " << name << "!");
      return false;
    }

    // determine config chapter
    SSSET("Plant_" << _i);
    char *pchapter;
    if (space < 0) {
      pchapter = new char[ss.str().size() + 1];
      std::strcpy(pchapter, ss.str().c_str());
    } else {
      pchapter = &name[space+1];
    }

    int pdim=0, mdim=0, adim=0;
    double dt=0;
    if (!base->init(pdim, mdim, adim, dt, fname, pchapter)) {
      EOUT("MultiPlant failed to initialize sub-plant " << name << " !");
      return false;
    } else
      IOUT("Initializing sub-plant " << name << " in chapter [" << pchapter << "]");

    _pstart.push_back(plant_state_dim);
    _mstart.push_back(measurement_dim);
    _astart.push_back(action_dim);
    _plants.push_back(base);

    plant_state_dim += pdim;
    measurement_dim += mdim;
    action_dim += adim;
    delta_t += dt;
  }

  if (_plants.size() < 1) {
    EOUT("No plant specified.");
    return false;
  }
  _pdim = _plants.size();

  if (plant_state_dim < 1 || measurement_dim < 1 || action_dim < 1) {
    EOUT("Invalid dimensionality.");
    return false;
  }

  return true;
}

bool MultiPlant::check_initial_state (double *initial_plant_state)
{
  for (_i=0; _i<_pdim; _i++)
    if (!_plants[_i]->check_initial_state (&initial_plant_state[_pstart[_i]]))
      return false;
  return true;
}

void MultiPlant::deinit ()
{
  for (_i=0; _i<_pdim; _i++)
    _plants[_i]->deinit();
}

void MultiPlant::notify_episode_starts ()
{
  for (_i=0; _i<_pdim; _i++)
    _plants[_i]->notify_episode_starts();
}

void MultiPlant::notify_episode_stops ()
{
  for (_i=0; _i<_pdim; _i++)
    _plants[_i]->notify_episode_stops();
}

void MultiPlant::notify_command_string (const char* buf)
{
  for (_i=0; _i<_pdim; _i++)
    _plants[_i]->notify_command_string (buf);
}

void MultiPlant::notify_suspend_for_aux_call_cmd ()
{
  for (_i=0; _i<_pdim; _i++)
    _plants[_i]->notify_suspend_for_aux_call_cmd();
}

void MultiPlant::notify_return_from_aux_call_cmd ()
{
  for (_i=0; _i<_pdim; _i++)
    _plants[_i]->notify_return_from_aux_call_cmd();
}


REGISTER_PLANT(MultiPlant, "Combines multiple plants into one.")
