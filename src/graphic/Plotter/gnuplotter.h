#ifndef _GNUPLOT_PLOTTER_GRAPHIC_H_
#define _GNUPLOT_PLOTTER_GRAPHIC_H_

#include "graphic.h"
#include <fstream>

#define GNUPLOTTER_MAX_NUM_GRAPHS 20

/** 
  * \ingroup GRAPHIC
  * \author Thomas Lampe */
class GnuplotterGraphic : public Graphic {
 public:
  virtual bool notify(const double *state, const double *observed_state,const double *reference_input,
		      const double *action, const long cycle, const long episode, const float total_time, 
		      const float episode_time, const long total_num_of_cycles);
  virtual bool init(int _state_dim, int _observation_dim, int _action_dim, int _reference_input_dim, double _delta_t, const char *fname=0);
  virtual bool deinit();
  
 protected:
  int _i, _dim, _dims[GNUPLOTTER_MAX_NUM_GRAPHS];
  double _scale[GNUPLOTTER_MAX_NUM_GRAPHS], _off[GNUPLOTTER_MAX_NUM_GRAPHS], _next;
  char _srcs[2*GNUPLOTTER_MAX_NUM_GRAPHS];
  ofstream _data_file;
};



#endif
