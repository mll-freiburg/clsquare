#ifndef _FRAMEVIEW_PLOTTER_GRAPHIC_H_
#define _FRAMEVIEW_PLOTTER_GRAPHIC_H_

#include "graphic.h"
#include "udpsocket.h" // required for communication with frameview
#include <sstream>

#define MAX_NUM_GRAPHS 6

/** 
  * \ingroup GRAPHIC
  * \author Thomas Lampe */
class FrameviewPlotterGraphic : public Graphic {
 public:
  virtual bool notify(const double *state, const double *observed_state,const double *reference_input,
		      const double *action, const long cycle, const long episode, const float total_time, 
		      const float episode_time, const long total_num_of_cycles);
  virtual bool init(int _state_dim, int _observation_dim, int _action_dim, int _reference_input_dim, double _delta_t, const char *fname=0);
  virtual bool deinit();
  
 protected:
  int port;  
  char hostname[500];
  UDPsocket sock;
  std::stringstream buffer;

  int _i, _dim, _dims[MAX_NUM_GRAPHS], _color[MAX_NUM_GRAPHS];
  double _step, _last[MAX_NUM_GRAPHS], _next, _scale[MAX_NUM_GRAPHS], _off[MAX_NUM_GRAPHS];
  char _srcs[2*MAX_NUM_GRAPHS];
//  enum {State, Observation, Action, Reference} mode;
};



#endif
