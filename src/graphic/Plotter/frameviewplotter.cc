#include <stdio.h>
#include <string.h>
#include <sstream>
#include <iomanip>
#include <cmath>
#include "valueparser.h"
#include "frameviewplotter.h"

bool FrameviewPlotterGraphic::notify(const double *state, const double *observed_state,const double *reference_input, 
			     const double *action, const long cycle, const long episode, const float total_time, 
			     const float episode_time, const long total_num_of_cycles)
{
  buffer.str("");
  buffer << setfill('0');
  for (_i=0; _i<_dim; _i++) {
    switch (_srcs[_i]) {
      case 'a':
        _next = action[_dims[_i]];
        break;
      case 's':
        _next = state[_dims[_i]];
        break;
      case 'o':
        _next = observed_state[_dims[_i]];
        break;
      case 'r':
        _next = reference_input[_dims[_i]];
        break;
    }

    buffer << "INS 1 LINE col=" << std::hex << setw(6) << _color[_i] << std::dec << setw(1) <<
              " (" << _step*cycle << "," << _last[_i]*_scale[_i]+_off[_i] << "," << _step*(cycle+1) << "," << _next*_scale[_i]+_off[_i] << ");";
    buffer << "MOV 1 (" << -_step*cycle << ",0); SL \"(cyc= "<< cycle << " seq=" << episode << " lap="<< episode_time << " total=" << total_time << " \";",
    _last[_i] = _next;
  }
  sock.send_msg(buffer.str().c_str(), buffer.str().size());
  return true;
}

bool FrameviewPlotterGraphic::init(int _state_dim, int _observation_dim, int _action_dim, int _reference_input_dim, double _delta_t, const char *fname)
{
  std::stringstream out;
  ValueParser vp(fname,"Graphic");

  // source vector for each graph
  char tmpsrc[2*MAX_NUM_GRAPHS];
  int tmplen = vp.get("sources", tmpsrc, 2*MAX_NUM_GRAPHS);
  _dim = 0;
  for (_i=0; _i<tmplen; _i+=2) {
    _srcs[_dim] = tmpsrc[_i];
    if (tmpsrc[_i]!='a' && tmpsrc[_i]!='o' && tmpsrc[_i]!='s' && tmpsrc[_i]!='r') {
      EOUT("Invalid source specification: " << tmpsrc);
      return false;
    }
    _dim++;
  }

  // vector indices
  if (vp.get("indices", _dims, _dim) < _dim) {
    EOUT("Parameter \"indices\" should have " << _dim << " entries.");
    return false;
  }

  // graph scaling
  for (_i=0; _i<_dim; _i++)
    _scale[_i] = 1.;
  vp.get("scaling", _scale, _dim);
  for (_i=0; _i<_dim; _i++)
    _scale[_i] *= 6. / _dim;

  for (_i=0; _i<_dim; _i++)
    _off[_i] = -12. * ((_i+1.) / (_dim+1.) - 0.5) - 3. /_dim;

  // colors
  for (_i=0; _i<_dim; _i++)
    _color[_i] = 0;
  vp.get("colors", _color, _dim);

  // communication
  int port;
  vp.get("port", port, 6010);

  char hostname[500];
  sprintf(hostname,"localhost");
  vp.get("hostname", hostname, 500);

  sock.init_socket_fd();
  sock.init_serv_addr(hostname, port);

  // first point
  vp.get("step", _step, .01);

  out << "VA (-5,0,12,8); EMP 0; INS 0 FRAME id=1; ";
  for (_i=0; _i<_dim; _i++)
    out << "INS 1 LINE col=" << std::hex << setfill('0') << setw(6) << _color[_i] << std::dec << setw(1) << " (0," << _off[_i] << "," << _step << "," << _off[_i] << "); ";
  sock.send_msg(out.str().c_str(), out.str().length());

  for (_i=0; _i<_dim; _i++)
    _last[_i] = 0.;

  return true;
}

bool FrameviewPlotterGraphic::deinit()
{
  sock.close_socket_fd();
  return true;
}

REGISTER_GRAPHIC(FrameviewPlotterGraphic, "visualizes the state of the cart pole")
