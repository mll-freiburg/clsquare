#include <stdio.h>
#include <string.h>
#include <sstream>
#include <stdlib.h>
#include "valueparser.h"
#include "gnuplotter.h"
#include "unistd.h"

bool GnuplotterGraphic::notify (const double *state, const double *observed_state,const double *reference_input, 
			     const double *action, const long cycle, const long episode, const float total_time, 
			     const float episode_time, const long total_num_of_cycles)
{
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
    _data_file << _next << " ";
  }
  _data_file << std::endl;
  return true;
}

bool GnuplotterGraphic::init (int _state_dim, int _observation_dim, int _action_dim, int _reference_input_dim, double _delta_t, const char *fname)
{
  std::stringstream out;
  ValueParser vp(fname, "Graphic");

  // source vector for each graph
  char tmpsrc[2*GNUPLOTTER_MAX_NUM_GRAPHS];
  int tmplen = vp.get("sources", tmpsrc, 2*GNUPLOTTER_MAX_NUM_GRAPHS);
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

  // gnuplot params
#define GET_PARAM_LIST(_param_,_default_) \
  int _param_[_dim]; \
  for (_i=0; _i<_dim; _i++) \
    _param_[_i] = _default_; \
  vp.get(#_param_, _param_, _dim);
  GET_PARAM_LIST(line_color, 0);
  GET_PARAM_LIST(line_type,  1);
  GET_PARAM_LIST(line_width, 1);

  // open files
#define OPEN_PLOT_FILE(_type_,_extension_); \
  char _type_##file[255]; \
  sprintf(_type_##file, "/tmp/gnuplotter."_extension_); \
  vp.get(#_type_"_file", _type_##file, 255); \
  _##_type_##_file.open(_type_##file); \
  if (!_##_type_##_file.good()) { \
    EOUT("Could not open "#_type_" file " << _type_##file << " for writing."); \
    return false; \
  }
  ofstream _script_file, _replot_file;
  OPEN_PLOT_FILE(script, "gp");
  OPEN_PLOT_FILE(replot, "gpr");
  OPEN_PLOT_FILE(data, "dat");

  // generate main script
if (vp.get("custom_script", scriptfile, 255) > 1) {
_script_file.close();
} else {
  _script_file << "set terminal x11 noraise" << std::endl;
  double range[2];
#define GET_RANGE(_dim_) \
  if (vp.get(#_dim_"range", range, 2) == 2) \
    _script_file << "set " << #_dim_ << "range [" << range[0] << ":" << range[1] << "]" << std::endl;
  GET_RANGE(x);
  GET_RANGE(y);
  int window;
  vp.get("window_size", window, 10);
  _script_file << "plot ";
  char title[255], par[255];
  for (_i=0; _i<_dim; _i++) {
    if (_i != 0) _script_file << ", \\" << std::endl;
    _script_file << "\"< tail -n " << window << " " << datafile << "\" u " << (_i+1) << " w lines lc " << line_color[_i] << " lt " << line_type[_i] << " lw " << line_width[_i];
    sprintf(par, "title_%d", _i);
    if (vp.get(par, title, 255) > 0)
      _script_file << " t \"" << title << "\"";
    else
      _script_file << " notitle";
  }
  _script_file << std::endl << "load \"" << replotfile << "\"";
  _script_file.close();
}

  // generate replot script
  double update;
  vp.get("update_freq", update, 0.1);
  _replot_file << "pause " << update << std::endl << "replot" << std::endl << "reread";
  _replot_file.close();

  sleep(1);
  char syscom[355];
  sprintf(syscom, "xterm -e \"gnuplot %s\" &", scriptfile);
  int ret = system(syscom);
  if (ret != 0)
    WOUT(10, "Xterm returned status code " << ret << ". Gnuplot may not have been started properly.");

  return true;
}

bool GnuplotterGraphic::deinit()
{
  _data_file.close();
  return true;
}

REGISTER_GRAPHIC(GnuplotterGraphic, "Plots states or actions using gnuplot.")
