/*
 clsquare - closed loop simulation system
 Copyright (c) 2004, Neuroinformatics Group, Prof. Dr. Martin Riedmiller,
 University of Osnabrueck
 Copyright (c) 2011, Machine Learning Lab, Prof. Dr. Martin Riedmiller,
 University of Freiburg

 Author: Martin Riedmiller, Roland Hafner, Sascha Lange

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are
 met:

 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in
 the documentation and/or other materials provided with the
 distribution.
 * Neither the name of the <ORGANIZATION> nor the names of its
 contributors may be used to endorse or promote products derived
 from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 DAMAGE.
 */

#include "mainloop.h"

#include <sys/types.h> // needed for pipe
#include <sys/stat.h> // needed for pipe
#include <fcntl.h> // needed for pipe


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "valueparser.h"
#include "defaultgraphic.h"
#include "registry.h"
#include "global.h"
#include <csignal>
#include "unistd.h"

#define SEPLINE "\n------------------------------------------------------------------\n"
#define MYASSERT(__CONSTRAINT__) {if (!(__CONSTRAINT__)){ EOUT("Constraint [" << #__CONSTRAINT__ << "] failed!"); return false; }}


#define PIPE_PAUSE 2
#define PIPE_EMPTY 0
#define PIPE_START 1
#define PIPE_STOP 3
#define PIPE_CONTINUE 4

#define PIPENAME "/tmp/pipe2cls"


MainLoop::MainLoop()
{
  plant       = 0;
  observer    = 0;
  controller  = 0;
  output      = 0;
  reward      = 0;
  graphic     = 0;
  statistics  = 0;
  input       = 0;
}

MainLoop::~MainLoop()
{
  ;
}

bool MainLoop::init()
{
  gettimeofday(&this->tv_prog_start,0);

  if (!read_options())     return false;
  if (!create_modules())   return false;
  if (!init_modules())     return false;
  if (!sys.alloc())        return false;
  if (spec.interactive_mode == true) {

    IOUT("Interactive Mode activated. Ready to get commands from pipe.");
    unlink(PIPENAME); /*Falls  schon vorhanden ist*/
    mkfifo(PIPENAME, 0666);

    if ((pipe2cls=open(PIPENAME,O_NONBLOCK)) == - 1) {
      EOUT("Error: can't open the pipe2cls.....\n");
      exit(0);
    }
  }
  return true;
}

void MainLoop::deinit()
{
  if (spec.interactive_mode) {
    close(pipe2cls);
    unlink(PIPENAME); /*Falls  schon vorhanden ist*/
  }
  this->deinit_and_destroy_modules();
  sys.free();
}

bool MainLoop::handle_cmd_line(int argc, char** argv)
{
  if (argc <2) {
    IOUT(SEPLINE << "Usage: " << argv[0] << " <params.cls> [<random seed>] \n"
         << "To get help: " << argv[0] << " --help \n"
         << "To get a list of available   plant       modules: " << argv[0] << " --list_plants \n"
         << "To get a list of available   controller  modules: " << argv[0] << " --list_controllers \n"
         << "To get a list of available   reward      modules: " << argv[0] << " --list_rewards \n"
         << "To get a list of available   graphic     modules: " << argv[0] << " --list_graphics \n"
         << "To get a list of available   statistics  modules: " << argv[0] << " --list_statistics\n"
         << "To get a list of available   observer    modules: " << argv[0] << " --list_observers \n"
         << "To get a list of available   input       modules: " << argv[0] << " --list_inputs \n"
         << "To get a list of available   output      modules: " << argv[0] << " --list_outputs"
         << SEPLINE);
    return false;
  }

  if (strcmp(argv[1], "--list_plants") == 0) {
    std::vector< std::string > names;
    std::vector< std::string > desc;
    PlantFactory::getThePlantFactory()->listEntries( &names , &desc );
    ISTREAM << SEPLINE;
    ISTREAM << "Available plant modules ("<< names.size()<<")\n";
    for (unsigned int i=0; i<names.size(); i++) {
      ISTREAM << " \nName: " << names[i] << "\n";
      ISTREAM << desc[i]  << "\n";
    }
    ISTREAM << SEPLINE;
    return false;
  } else if (strcmp(argv[1], "--list_controllers") == 0) {
    std::vector< std::string > names;
    std::vector< std::string > desc;
    ControllerFactory::getTheControllerFactory()->listEntries( &names , &desc );
    ISTREAM << SEPLINE;
    ISTREAM << "Available controller modules ("<< names.size()<<")\n";
    for (unsigned int i=0; i<names.size(); i++) {
      ISTREAM << " \nName: " << names[i] << "\n";
      ISTREAM << desc[i]  << "\n";
    }
    ISTREAM << SEPLINE;
    return false;
  } else if (strcmp(argv[1], "--list_rewards") == 0) {
    std::vector< std::string > names;
    std::vector< std::string > desc;
    RewardFactory::getTheRewardFactory()->listEntries( &names , &desc );
    ISTREAM << SEPLINE;
    ISTREAM << "Available reward modules ("<< names.size()<<")\n";
    for (unsigned int i=0; i<names.size(); i++) {
      ISTREAM << " \nName: " << names[i] << "\n";
      ISTREAM << desc[i]  << "\n";
    }
    ISTREAM << SEPLINE;
    return false;
  } else if (strcmp(argv[1], "--list_graphics") == 0) {
    std::vector< std::string > names;
    std::vector< std::string > desc;
    GraphicFactory::getTheGraphicFactory()->listEntries( &names , &desc );
    ISTREAM << SEPLINE;
    ISTREAM << "Available graphic modules ("<< names.size()<<")\n";
    for (unsigned int i=0; i<names.size(); i++) {
      ISTREAM << " \nName: " << names[i] << "\n";
      ISTREAM << desc[i]  << "\n";
    }
    ISTREAM << SEPLINE;
    return false;
  } else if (strcmp(argv[1], "--list_statistics") == 0) {
    std::vector< std::string > names;
    std::vector< std::string > desc;
    StatisticsFactory::getTheStatisticsFactory()->listEntries( &names , &desc );
    ISTREAM << SEPLINE;
    ISTREAM << "Available statistics modules ("<< names.size()<<")\n";
    for (unsigned int i=0; i<names.size(); i++) {
      ISTREAM << " \nName: " << names[i] << "\n";
      ISTREAM << desc[i]  << "\n";
    }
    ISTREAM << SEPLINE;
    return false;
  } else if (strcmp(argv[1], "--list_observers") == 0) {
    std::vector< std::string > names;
    std::vector< std::string > desc;
    ObserverFactory::getTheObserverFactory()->listEntries( &names , &desc );
    ISTREAM << SEPLINE;
    ISTREAM << "Available observer modules ("<< names.size()<<")\n";
    for (unsigned int i=0; i<names.size(); i++) {
      ISTREAM << " \nName: " << names[i] << "\n";
      ISTREAM << desc[i]  << "\n";
    }
    ISTREAM << SEPLINE;
    return false;
  } else if (strcmp(argv[1], "--list_inputs") == 0) {
    std::vector< std::string > names;
    std::vector< std::string > desc;
    InputFactory::getTheInputFactory()->listEntries( &names , &desc );
    ISTREAM << SEPLINE;
    ISTREAM << "Available input modules ("<< names.size()<<")\n";
    for (unsigned int i=0; i<names.size(); i++) {
      ISTREAM << " \nName: " << names[i] << "\n";
      ISTREAM << desc[i]  << "\n";
    }
    ISTREAM << SEPLINE;
    return false;
  } else if (strcmp(argv[1], "--list_outputs") == 0) {
    std::vector< std::string > names;
    std::vector< std::string > desc;
    OutputFactory::getTheOutputFactory()->listEntries( &names , &desc );
    ISTREAM << SEPLINE;
    ISTREAM << "Available output modules ("<< names.size()<<")\n";
    for (unsigned int i=0; i<names.size(); i++) {
      ISTREAM << " \nName: " << names[i] << "\n";
      ISTREAM << desc[i]  << "\n";
    }
    ISTREAM << SEPLINE;
    return false;
  } else if (strcmp(argv[1], "--help") == 0) {
    std::cout << SEPLINE;
    this->get_help(std::cout);
    std::cout << "\n" << SEPLINE;
    return false;
  }

  this->spec.config_fname = argv[1];

  if (argc == 3) {
    long tmp_seed = atol(argv[2]);
    IOUT("setting seed to " << tmp_seed );
    srand48(tmp_seed);
  }

  return true;
}

int MainLoop::check_pipe()
{
  int result = PIPE_EMPTY;
  int p;

  char buf[1024];
  do {
    // setting up select for unblocked input
    fd_set rfds;
    struct timeval tv;

    FD_ZERO(&rfds);
    FD_SET(pipe2cls, &rfds); // add filedescriptor pipe2cls to set
    //  tv.tv_sec = 10; tv.tv_usec = 0;  // set max. waiting time; 10s only for test
    tv.tv_sec = 0;
    tv.tv_usec = 1;  // no waiting, only if something is in pipe
    if (select(pipe2cls + 1, &rfds, NULL, NULL, &tv) >0) { // pipe is not empty
      int dummy = read(pipe2cls, &buf, sizeof(buf));
      buf[dummy] = '\0'; // sender may not send string terminal
      for (p=0; p<dummy; p++) if (buf[p] == '\n') buf[p] = '\0'; // newlines are evil
      if (strlen(buf)>0) {
        IOUTV(spec.verbosity,2, "In episode: "<<sys.current_time.episode_ctr
              <<" got command via pipe: " << buf );
      }
      if (strncmp(buf,"start",5) == 0)
        result = PIPE_START;
      else if (strncmp(buf,"stop",4) == 0)
        result = PIPE_STOP;
      else if (strncmp(buf,"pause",5) == 0) {
        result = PIPE_PAUSE;
        plant->notify_command_string("plant_cmd pause");
      } else if (strncmp(buf,"plant_cmd",9) == 0) {
        plant->notify_command_string(buf);
      } else if (strncmp(buf,"graphic_cmd",11) == 0) {
        graphic->notify_command_string(buf);
      } else if (strncmp(buf,"controller_cmd",14) == 0) {
        controller->notify_command_string(buf);
      } else {
        //WOUT(10, "Received invalid command \"" << buf << "\" via pipe.");
      }
      sprintf(buf,"%s","\0");
      dummy = write(pipe2cls, &buf, sizeof(buf));
    }
  } while (result == PIPE_PAUSE); // pause

  return result;
}


bool MainLoop::read_options()
{
  char buf[MAX_STR_LEN];
  ValueParser vp(spec.config_fname.c_str(),"Main");

  vp.get("verbosity",           spec.verbosity);
  vp.get("interactive_mode",    spec.interactive_mode);
  vp.get("num_episodes",        spec.num_episodes);
  MYASSERT(spec.num_episodes > 0)
  vp.get("sleep_every_cycle",   spec.sleep_every_cycle);
  vp.get("cycles_per_episode",  spec.cycles_per_episode);
  MYASSERT(spec.cycles_per_episode >= 0)
  vp.get("call_cmd_freq",       spec.call_cmd_freq);
  if (vp.get("call_cmd",        buf ,MAX_STR_LEN) >= 0) spec.call_cmd = buf;
  vp.get("max_initial_retries", spec.max_initial_retries);
  vp.get("max_external_signal_retries",  spec.max_external_signal_retries);

  vp.get("do_not_start_in_terminal_states",
         spec.do_not_start_in_terminal_states, 1);

  if (vp.get("plant",buf,MAX_STR_LEN) <= 0) {
    EOUT ("No configuration entry found for module 'plant'!");
    return false;
  }
  spec.plant_module_name = buf;

  if (vp.get("controller",buf,MAX_STR_LEN) <= 0) {
    EOUT ("No configuration entry found for module 'controller'!");
    return false;
  }
  spec.controller_module_name = buf;

  if (vp.get("reward", buf, MAX_STR_LEN) <= 0) {
    // WOUT ( 9, "No configuration entry found for module 'reward'! Thus, using StandardReward.");
    spec.reward_module_name = "StandardReward";
  } else {
    spec.reward_module_name = buf;
  }

  if (vp.get("observer",buf,MAX_STR_LEN) <= 0) {
    spec.observer_module_name = "DefaultObserver";
    WOUT( 9 , "No observer module given, using default (nop): " << spec.observer_module_name );
  } else
    spec.observer_module_name = buf;

  if (vp.get("graphic",buf,MAX_STR_LEN) <= 0) {
    spec.graphic_module_name = ""; //"DefaultGraphic";
    // WOUT( 9 , "No graphic module given, using default (nop): " << spec.graphic_module_name );
  } else
    spec.graphic_module_name = buf;

  if (vp.get("statistics",buf,MAX_STR_LEN) <= 0) {
    spec.statistics_module_name = "NOOPStatistics";
    WOUT( 9 , "No statistics module given, using default (nop): " << spec.statistics_module_name );
  } else
    spec.statistics_module_name = buf;

  if (vp.get("input",buf,MAX_STR_LEN) <= 0) {
    spec.input_module_name = "DefaultInput";
  } else
    spec.input_module_name = buf;

  if (vp.get("output",buf,MAX_STR_LEN) <= 0) {
    spec.output_module_name = "DefaultOutput";
  } else
    spec.output_module_name = buf;

  return true;
}

bool MainLoop::create_modules()
{
  IOUTV(spec.verbosity, 2 , SEPLINE << "creating modules ... ");

  plant = PlantFactory::getThePlantFactory()->create(spec.plant_module_name.c_str());
  if (plant == 0) {
    EOUT ("No implementation found for plant: (" << spec.plant_module_name << ")!");
    return false;
  }
  controller = ControllerFactory::getTheControllerFactory()->create(spec.controller_module_name.c_str());
  if (controller == 0) {
    EOUT ("No implementation found for Controller (" << spec.controller_module_name << ")!");
    return false;
  }

  if (spec.reward_module_name.compare("use_plant") == 0) {
    reward = dynamic_cast<Reward*>(plant);
    if (!reward) {
      EOUT ("Plant can not be used as the reward module since it doesn't implement the Reward interface.");
      return false;
    }
  } else {
    reward = RewardFactory::getTheRewardFactory()->create(spec.reward_module_name.c_str());
    if (!reward) {
      EOUT ("No implementation found for Reward (" << spec.reward_module_name << ")!");
      return false;
    }
  }
  if (spec.graphic_module_name.compare("") == 0) {
    spec.graphic_module_name = plant->get_default_graphics();
    if (spec.graphic_module_name.compare("DefaultGraphic") != 0)
      IOUT("No graphic module given, using plant default: " << spec.graphic_module_name );
  }
  graphic = GraphicFactory::getTheGraphicFactory()->create(spec.graphic_module_name.c_str());
  if (graphic == 0) {
    EOUT ("No implementation found for graphic: (" << spec.graphic_module_name << ")!");
    return false;
  }

  observer = ObserverFactory::getTheObserverFactory()->create(spec.observer_module_name.c_str());
  if (observer == 0) {
    EOUT ("No implementation found for observer: (" << spec.observer_module_name << ")!");
    return false;
  }

  this->statistics = StatisticsFactory::getTheStatisticsFactory()->create(spec.statistics_module_name.c_str());
  if (this->statistics == 0) {
    EOUT ("No implementation found for statistics: (" << spec.statistics_module_name << ")!");
    return false;
  }

  this->input = InputFactory::getTheInputFactory()->create(spec.input_module_name.c_str());
  if (this->input == 0) {
    EOUT ("No implementation found for input: (" << spec.input_module_name << ")!");
    return false;
  }

  this->output = OutputFactory::getTheOutputFactory()->create(spec.output_module_name.c_str());
  if (this->output == 0) {
    EOUT ("No implementation found for output: (" << spec.output_module_name << ")!");
    return false;
  }

  IOUTV( spec.verbosity , 2 , " All modules successfully created!" << SEPLINE);
  return true;
}

bool MainLoop::init_modules()
{
  this->sys.current_time.total_real_time = this->get_time();
  IOUTV( spec.verbosity , 2, SEPLINE << "init of modules ...")
  if (!plant->init_main( sys.dim.plant_state_dim, sys.dim.measurement_dim,
                         sys.dim.action_dim, sys.dim.external_signal_dim,
                         sys.current_time.delta_t, spec.config_fname.c_str()) ) {
    EOUT ("Init of plant: (" << spec.plant_module_name << ") failed!");
    return false;
  }
  MYASSERT( sys.dim.plant_state_dim > 0 );
  MYASSERT( sys.dim.action_dim > 0 );
  MYASSERT( sys.dim.measurement_dim > 0 );
  if (!observer->init(sys.dim.plant_state_dim,
                      sys.dim.measurement_dim, sys.dim.action_dim, sys.dim.observed_state_dim, spec.config_fname.c_str())) {
    EOUT ("Init of observer: (" << spec.observer_module_name << ") failed!");
    return false;
  }
  MYASSERT( sys.dim.observed_state_dim > 0 );
  if (!reward->init(sys.dim.plant_state_dim, sys.dim.measurement_dim, sys.dim.observed_state_dim, sys.dim.action_dim, &spec.reward_type, spec.config_fname.c_str())) {
    EOUT ("Init of reward: (" << spec.reward_module_name << ") failed!");
    return false;
  }
  if (spec.reward_type == REWARD_INPUT_UNSPECIFIED) {
    EOUT("The init method of the selected reward module did not sepcify which input representation (plant_state, measurement, observed_state) to use.");
    return false;
  }

  if (!controller->init( sys.dim.observed_state_dim, sys.dim.action_dim, sys.current_time.delta_t, spec.config_fname.c_str())) {
    EOUT ("Init of controller: (" << spec.controller_module_name << ") failed!");
    return false;
  }
  if (!input->init(sys.dim.plant_state_dim, sys.dim.action_dim, sys.dim.external_signal_dim, sys.current_time.delta_t, spec.config_fname.c_str())) {
    EOUT("Init of module input failed!");
    return false;
  }
  //if(!output->init(sys.dim.plant_state_dim, sys.dim.measurement_dim, sys.dim.action_dim, sys.dim.external_signal_dim, sys.current_time.delta_t, spec.config_fname.c_str()))
  if (!output->init(sys.dim , sys.current_time , spec.config_fname.c_str())) {
    EOUT("Init of module output failed!");
    return false;
  }
  if (!graphic->init(sys.dim.plant_state_dim, sys.dim.observed_state_dim, sys.dim.action_dim, sys.dim.external_signal_dim, sys.current_time.delta_t, spec.config_fname.c_str())) {
    EOUT ("Init of graphic: ("<< spec.graphic_module_name <<") failed!");
    return false;
  }
  if (!statistics->init(sys.dim.plant_state_dim, sys.dim.measurement_dim, sys.dim.observed_state_dim, sys.dim.action_dim, sys.current_time.delta_t, spec.cycles_per_episode, spec.config_fname.c_str())) {
    EOUT("Init of module statistic failed!");
    return false;
  }

  IOUTV(spec.verbosity, 1, "All modules successfully initialized"
        << "\n    plant_state_dim     : " << sys.dim.plant_state_dim
        << "\n    measurement_dim     : " << sys.dim.measurement_dim
        << "\n    observation_dim     : " << sys.dim.observed_state_dim
        << "\n    action_dim          : " << sys.dim.action_dim
        << "\n    external_signal_dim : " << sys.dim.external_signal_dim
        << "\n    delta_t             : " << sys.current_time.delta_t
        << SEPLINE);

  return true;
}

#define DEINITANDDELETE( __module__ ) if ( __module__!=0 ) { __module__->deinit(); delete __module__; __module__=0;}

void MainLoop::deinit_and_destroy_modules()
{
  DEINITANDDELETE( plant );
  DEINITANDDELETE( observer );
  DEINITANDDELETE( controller );
  DEINITANDDELETE( input );
  DEINITANDDELETE( output );
  DEINITANDDELETE( reward );
  DEINITANDDELETE( graphic );
  DEINITANDDELETE( statistics );
}

void MainLoop::get_initial_state()
{
  bool      accepted  = true;
  long int  patience  = 0;

  // init the initial state to zero (default)
  for (int i=0; i<sys.dim.plant_state_dim; i++) sys.current_state_vars.plant_state[i]=0.0;

  // try to set an initial state. if plant or controller refuses, get another one.
  do {
    accepted  = true;
    input->get_initial_episode_plant_state(sys.current_state_vars.plant_state, sys.current_time.episode_ctr);
    if (!plant->check_initial_state(sys.current_state_vars.plant_state)) {
      IOUTV(spec.verbosity,0, "Plant refused initial state, try another one ..."  << patience );
      accepted = false;
      continue;
    }
    plant->get_measurement(sys.current_state_vars.plant_state, sys.current_state_vars.external_signal, sys.current_state_vars.measurement);
    observer->get_observed_state(sys.prev_state_vars.measurement, sys.prev_action,
                                 sys.current_state_vars.measurement, sys.current_time.cycle_ctr,
                                 sys.current_state_vars.observed_state);

    if (spec.do_not_start_in_terminal_states) { // check state with reward module (default setting: reject terminal states)
      double *current = sys.current_state_vars.measurement; // the default is to use measurements with the reward module
      if (spec.reward_type == REWARD_INPUT_PLANT_STATE) {  // configured to use plant states
        current = sys.current_state_vars.plant_state;
      } else if (spec.reward_type == REWARD_INPUT_OBSERVED_STATE) { // configured to use observations
        current = sys.current_state_vars.observed_state;
      }

      if (reward->is_terminal(current)) {
        IOUTV(spec.verbosity,1, "Mainloop refused initial state or measurment because it is a terminal state. Try another one..." << patience);
        accepted = false;
        continue ;
      }
    }
    if (!controller->check_initial_state(sys.current_state_vars.observed_state, sys.dim.observed_state_dim)) {
      IOUTV(spec.verbosity,1, "Controller refused initial state or measurement, try another one ..." << patience );
      accepted = false;
    }
  } while (accepted == false && patience++ < spec.max_initial_retries-1);

  if (accepted == false) {
    CLSERR("Cannot find a reasonable initial state.");
  }
}

void MainLoop::get_external_signal(double* external_signal, long cycle_ctr)
{
  // init ref to zero (default)
  for (int i=0; i<sys.dim.external_signal_dim; i++) external_signal[i]=0.0;
  int i=0;
  do {
    input->get_external_signal(external_signal , cycle_ctr);
    if (plant->check_external_signal(external_signal)) return;
    i++;
  } while (i<this->spec.max_external_signal_retries);
  CLSERR("Could not find appropriate external signal after: " << i << " trails.");
}

#define TIMESHIFTSWAPDBLPTR3( _prev_ , _curr_ , _next_ ) { double* swap = _prev_; _prev_=_curr_; _curr_=_next_; _next_=swap;}
#define TIMESHIFTSWAPDBLPTR2( _prev_ , _curr_ ) { double* swap = _prev_; _prev_=_curr_; _curr_=swap;}

void MainLoop::do_episode()
{
  // init episode
  sys.current_time.cycle_ctr     = 1;
  sys.current_time.episode_time  = 0;

  get_external_signal( sys.current_state_vars.external_signal , sys.current_time.cycle_ctr  );
  get_initial_state();

  sys.current_time.total_real_time = this->get_time();

  plant->notify_episode_starts();
  observer->notify_episode_starts();
  controller->notify_episode_starts();
  statistics->notify_episode_starts(sys.current_time.episode_ctr);
  output->notify_episode_starts(sys.current_time);
  graphic->notify_episode_starts(sys.current_time.episode_ctr);


  bool break_request = false;
  do {
    if (sys.current_time.cycle_ctr >= spec.cycles_per_episode) {
      break_request = true;
      IOUTV( spec.verbosity , 1 , " Episode break request: max cycles reached!" << SEPLINE);
    }

    observer->get_observed_state(sys.prev_state_vars.measurement, sys.prev_action, sys.current_state_vars.measurement,
                                 sys.current_time.cycle_ctr, sys.current_state_vars.observed_state);

    bool tmp_break_request = ! controller->get_action(sys.current_state_vars.observed_state,
                             sys.current_action );
    break_request |= tmp_break_request;
    if (tmp_break_request) {
      IOUTV( spec.verbosity , 1 , " Episode break request: Controller requests break!" << SEPLINE);
    }

    /*
    break_request |= ! controller->get_action(sys.current_state_vars.observed_state,
                                              sys.current_action );
    */

    // start determining reward
    double prev_reward = 0, terminal_reward = 0;
    bool is_terminal_state = false;
    {
      double *prev = sys.prev_state_vars.measurement;  // the mainloop will pass the state representation to the reward module it
      double *current = sys.current_state_vars.measurement; // requested in the init method. the default is to use measurements.

      if (spec.reward_type == REWARD_INPUT_PLANT_STATE) {
        prev = sys.prev_state_vars.plant_state;
        current = sys.current_state_vars.plant_state;
      } else if (spec.reward_type == REWARD_INPUT_OBSERVED_STATE) {
        prev = sys.prev_state_vars.observed_state;
        current = sys.current_state_vars.observed_state;
      }

      if (sys.current_time.cycle_ctr>1) {
        prev_reward = reward->get_reward(prev, sys.prev_action, current);
      }
      if ((is_terminal_state = reward->is_terminal(current)) || break_request)
        terminal_reward = reward->get_terminal_reward(current);
    }
    break_request|= is_terminal_state; // if reward detects terminal state, episode is stopped

    // start notify_transition
    if (sys.current_time.cycle_ctr>1) {
      controller->notify_transition(sys.prev_state_vars.observed_state, sys.prev_action,
                                    sys.current_state_vars.observed_state, prev_reward, is_terminal_state,
                                    terminal_reward);
    }
    graphic->notify(sys.current_state_vars.plant_state, sys.current_state_vars.observed_state, sys.current_state_vars.external_signal,
                    sys.current_action,
                    sys.current_time.cycle_ctr, sys.current_time.episode_ctr,
                    sys.current_time.episode_time, sys.current_time.total_time, sys.current_time.total_num_of_cycles);
    output->notify( sys.current_state_vars , sys.current_action , sys.current_time , prev_reward);
    statistics->notify(sys.current_state_vars.plant_state, sys.current_state_vars.measurement,
                       sys.current_state_vars.observed_state, sys.current_action,
                       sys.current_time.cycle_ctr, sys.current_time.episode_ctr, sys.current_time.episode_time,
                       sys.current_time.total_time, sys.current_time.total_num_of_cycles, prev_reward);
    // end notify_transition

    int result_check_pipe = PIPE_EMPTY;
    if (spec.interactive_mode) {
      result_check_pipe = check_pipe();
      if (result_check_pipe == PIPE_STOP) {
        break_request = true; // notify all modules, that episode stops
      }
    }


    // start check4break
    if (break_request == true) {
      //plant, controller, reward or cycle_ctr triggered stop of episode
      plant->notify_episode_stops();
      observer->notify_episode_stops();
      output->notify_episode_stops( sys.current_state_vars , sys.current_time , is_terminal_state , terminal_reward );
      statistics->notify_episode_stops(sys.current_state_vars.plant_state, sys.current_state_vars.measurement,
                                       sys.current_state_vars.observed_state,
                                       sys.current_time.cycle_ctr, sys.current_time.episode_ctr, sys.current_time.episode_time,
                                       sys.current_time.total_time, sys.current_time.total_num_of_cycles,
                                       is_terminal_state, terminal_reward);
      graphic->notify_episode_stops(sys.current_state_vars.plant_state, sys.current_state_vars.measurement,
                                    sys.current_state_vars.external_signal,
                                    sys.current_time.cycle_ctr, sys.current_time.episode_ctr,
                                    sys.current_time.episode_time, sys.current_time.total_time, sys.current_time.total_num_of_cycles);
      // last directive in episode as learning can interrupt program flow here
      // (for several minutes up to hours)
      controller->notify_episode_stops(sys.current_state_vars.observed_state);
      if (result_check_pipe == PIPE_STOP) {
        CLSQUIT("Request to stop by pipe command, will quit CLSQUARE now.");
      }
      if (spec.sleep_every_cycle>0) {
        usleep(spec.sleep_every_cycle *1000); // wait for the last time, e.g to display graphics
      }
      break; // stop episode
    }
    // end check4break

    sys.current_time.total_real_time = this->get_time();

    tmp_break_request = !plant->get_next_plant_state(sys.current_state_vars.plant_state,
                        sys.current_action, sys.current_state_vars.external_signal,
                        sys.next_state_vars.plant_state);

    break_request |= tmp_break_request;
    if (tmp_break_request) {
      IOUTV( spec.verbosity , 1 , " Episode break request: Plant requests break!" << SEPLINE);
    }

    get_external_signal( sys.next_state_vars.external_signal , sys.current_time.cycle_ctr+1  );
    plant->get_measurement(sys.next_state_vars.plant_state, sys.next_state_vars.external_signal, sys.next_state_vars.measurement);

    // start time shift. Just "move" the pointers one step forward in time; from hereafter prev points to the (formally) current state, current to next and next points to the now-unused buffer of prev.

    TIMESHIFTSWAPDBLPTR3( sys.prev_state_vars.plant_state      , sys.current_state_vars.plant_state       , sys.next_state_vars.plant_state );
    TIMESHIFTSWAPDBLPTR3( sys.prev_state_vars.measurement      , sys.current_state_vars.measurement       , sys.next_state_vars.measurement );
    TIMESHIFTSWAPDBLPTR3( sys.prev_state_vars.observed_state   , sys.current_state_vars.observed_state    , sys.next_state_vars.observed_state );
    TIMESHIFTSWAPDBLPTR3( sys.prev_state_vars.external_signal  , sys.current_state_vars.external_signal   , sys.next_state_vars.external_signal );
    TIMESHIFTSWAPDBLPTR2( sys.prev_action                      , sys.current_action );

    sys.current_time.cycle_ctr ++;
    sys.current_time.total_time += sys.current_time.delta_t;
    sys.current_time.episode_time += sys.current_time.delta_t;
    if (spec.sleep_every_cycle>0)
      usleep(spec.sleep_every_cycle *1000);
    // end time shift
  } while (true);  // loop is left by 'break'
}


void MainLoop::do_aux_call()
{
  if (spec.call_cmd_freq>0 && (sys.current_time.episode_ctr % spec.call_cmd_freq) == 0) {
    char call_cmd_tmp[MAX_STR_LEN + 20];
    plant->notify_suspend_for_aux_call_cmd();
    sprintf(call_cmd_tmp,"%s %10ld %10ld %10.2g",spec.call_cmd.c_str(), sys.current_time.episode_ctr, sys.current_time.total_num_of_cycles, sys.current_time.total_time);
    if (system(call_cmd_tmp) < 0) EOUT("Executing command: " << call_cmd_tmp << " seem to fail!");
    plant->notify_return_from_aux_call_cmd();
  }
}

void MainLoop::do_episodes()
{
  this->do_aux_call();
  for (sys.current_time.episode_ctr = 1; sys.current_time.episode_ctr <= spec.num_episodes; sys.current_time.episode_ctr ++) {
    this->do_episode();
    sys.current_time.total_num_of_cycles += sys.current_time.cycle_ctr;
    this->do_aux_call();
  }
}

void MainLoop::get_help(std::ostream& out)
{
  out
  << "Please read the CLSquare documentation in CLSquare/doc and the doxygen documentation under /CLSquare/doc/html for details.";
}

long double MainLoop::get_time()
{
  timeval tv_curr;
  gettimeofday(&tv_curr , 0);
  long double res_msec = (tv_curr.tv_sec - this->tv_prog_start.tv_sec) * 1000.0 + (tv_curr.tv_usec - this->tv_prog_start.tv_usec) / 1000.0;
  return res_msec/1000.0;
}


// ----------------------------------------------------------------------------------------------------------------------------
/** The main loop of CLSquare. */
// ----------------------------------------------------------------------------------------------------------------------------

MainLoop ml;

void sighandler(int sig)
{
  EOUT("Catched SIGABORT, will quit CLSQUARE now!");
  ml.deinit();
  exit(1);
}

int main(int argc, char **argv)
{

  // catch signals to allow for deinit of mainloop
  signal(SIGABRT, &sighandler);
  signal(SIGTERM, &sighandler);
  signal(SIGINT, &sighandler);

  if (!ml.handle_cmd_line(argc,argv))    return 0;

  try {
    if (!ml.init())  {
      EOUT(SEPLINE << "ATTENTION: The init phase failed." << SEPLINE);
      ml.deinit();
      return 1;
    }
  } catch (CLSquareException& e) {
    if (e.is_critical()) {
      EOUT(SEPLINE << "ATTENTION: Unexpected end of CLSquare init phase ... please don't use exceptions here!!");
      EOUT(e.what() << SEPLINE);
      exit(1); // IMPORTANT radical exit skips deinit, may leave plant running just because controller did not initialize
    } else {
      IOUT(SEPLINE << "End of CLSquare due to: ");
      IOUT(e.what() << SEPLINE);
    }
  } catch (std::exception& e) {
    EOUT(SEPLINE << "ATTENTION: Unexpected end of CLSquare init phase with unknown exception ...");
    EOUT(e.what() << SEPLINE);
    exit(1);
  }

  try {
    ml.do_episodes();
  } catch (CLSquareException& e) {
    if (e.is_critical()) {
      EOUT(SEPLINE << "ATTENTION: Unexpected end of CLSquare ... !!");
      EOUT(e.what() << SEPLINE);
    } else {
      IOUT(SEPLINE << "End of CLSquare due to: ");
      IOUT(e.what() << SEPLINE);
    }
  }
  // commented out during development phase since catching std-exceptions hides stack-traces e.g. due to bad_allocs
  catch (std::exception& e) {
    EOUT(SEPLINE << "ATTENTION: Unexpected end of CLSquare loop with unkonown exception ... may be caused by configuration mismatch or internal errors!");
    EOUT(e.what() << SEPLINE);
  }

  // after init phase the program flow will ensure that we deinit all modules
  // IMPORTANT for plant deinit if a real device has to be shut down properly
  ml.deinit();
}
// ----------------------------------------------------------------------------------------------------------------------------
