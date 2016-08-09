#ifndef _MAINLOOP_H_
#define _MAINLOOP_H_

/*! \mainpage CLSquare code documentation
 *
 * Authors: Martin Riedmiller, Roland Hafner, Sascha Lange, Stephan Timmer, Manuel Blum, Thomas Lampe \n
 * Machine Learning Lab \n
 * University of Freiburg \n
 *
 * \section intro_sec Introduction
 * \f$CLS^2\f$ (pronounced: CLSquare)  provides a standardized framework for testing  - not only - Reinforcement Learning
 *  agents (called \c controller in the following) in a growing number of different environments
 *  (called \c plant in the following). Special care is taken on the easy integration of new plants and controllers. By this,
 *  we want to encourage people to contribute to the system by sharing their plants and controllers.
 * \see CONTROLLER
 * \see PLANT
 *
 * \subsection features Features:
 *
 * \li provides a standarized control loop with built-in documentation facilities as used in typical Reinforcement Learning tasks
 * \li easy combination of different plants and controllers via definition in a configuration file
 * \li equal treatment of real and simulated plants
 * \li standardized, external graphical interface
 * \li lean interfaces for plants and controllers using standard data types only, allow a quick migration to and from \f$CLS^2\f$ from and to other software environments
 * \li prototypical implementations of plant and controller make the implementation of new
 systems easy
 * \li distinction between plant state and observed state allows implementation of POMDPs.
 *
 * \section install_sec Installation
 *
 * \subsection step1 Step1: Get a fresh copy of CLSquare
 *
 * At the moment we do not provide precompiled versions. So you have to get the newest sources from:
 * \li http://sourceforge.net/projects/clss/
 *
 * \subsection step2 Step2: Unpack the source code
 *
 * \li tar xzvf clsquare.tgz
 *
 * \subsection step3 Step3: Compile the main tree
 *
 * \li mkdir build
 * \li cd build
 * \li cmake ..
 * \li make install
 *
 * For an extended description of the base concepts behind \f$CLS^2\f$ and details
 * on advanced concepts and how to develop your own modules, please refer to the
 * whitepaper in \e doc/WhitePaper.
 */

#include "kerneldatatypes.h"
#include "output.h"
#include "reward.h"
#include "observer.h"
#include "statistics.h"
#include "input.h"
#include "plant.h"
#include "graphic.h"
#include "controller.h"

#include <iostream>
#include <string>
#include <sys/time.h>

/** @defgroup MAIN Main CLSquare Loop
  * The main CLSquare loop.
  */

/** Implementation of the main CLSquare loop.
  * For special cases this main loop may be reimplemented, using the provided sub-functionality.
  *
  * Config Entries @n
  * Place these config entries under the chapter [Main] in the config file:
  * \li num_episodes (long [1,]) @n number of episodes to run
  * \li cycles_per_episode (long [0,]) @n maximal number of cycles in each episode
  *
  * Optional Config Entries
  * \li verbosity  (int [0,]) @n verbosity level of outputs for the mainloop
  * \li interactive_mode (bool) @n pipe communication to CLSquare and modules enabled
  * \li max_initial_retries  (long [0,]) @n the number of attempts to get a valid initial plant state from the input module
  * \li sleep_every_cycle (long [0,]) @n number of milli seconds delay between two cycles (only for simulation, not for realtime plants!!)
  * \li call_cmd (string) @n a command that will be executed every call_cmd_freq episode
  * \li call_cmd_freq (int [0,]) @n frequency for the call_cmd
  *
  * @ingroup MAIN
  **/
class MainLoop {
public:
    MainLoop();
    ~MainLoop();
    /** Handle all command line arguments.
      * \param arg number of arguments
      * \param argv argument list
      */
    bool handle_cmd_line(int arg, char** argv);
    /** Initialize the mainloop and all modules */
    bool init();
    /** Process the required number of episodes */
    void do_episodes();
    /** Deinit the main loop and all modules.
      * \attention Deinit the plant module may be important for safety issues in real world plants.
      *            Please do not use exit calls in the code, instead use #CLSERR(__x__) macro to ensure that
      *            Pant::deinit will be called.
      */
    void deinit();

protected:
    /** Specification for the task.
      * This values are set by configuration entries in the Main section of the configuration file.
      */
    spec_struct spec;
    sys_struct  sys;
    int pipe2cls; // pipe for communication

    Plant       *plant;
    Observer    *observer;
    Controller  *controller;
    Output      *output;
    Reward      *reward;
    Graphic     *graphic;
    Statistics  *statistics;
    Input       *input;

    timeval     tv_prog_start;                  ///< the timestamp of the program start

    bool read_options();
    bool create_modules();
    bool init_modules();
    void deinit_and_destroy_modules();
    int check_pipe();

    void do_episode();
    void get_initial_state();
    void get_external_signal(double* external_signal , long cycle_ctr);

    void do_aux_call();

    void get_help(std::ostream& out);

    /** Helper function to get the current time (in s) since program start.**/
    long double get_time();
};

#endif
