#ifndef _QTABLE_CONTROLLER_H_
#define _QTABLE_CONTROLLER_H_

#include "controller.h"
#include "qtable.h"

/**
 * @ingroup CONTROLLER   
 * @ingroup LEARNING
 */
class QTableController : public Controller {
public:
  QTableController();
  virtual ~QTableController();

  bool get_action(const double* observed_state, double* action);
  bool init(const int observation_dim, const int action_dim, double deltat, const char* fname=0, const char* chapter=0);
  void deinit();
  void notify_transition(const double* observed_state,
                         const double* action,
                         const double* next_observed_state,
                         const double reward,
                         const bool is_terminal_state,
                         const double terminal_reward);
  void notify_episode_stops(const double* current_observed_state);

protected:
  QTable* qtable;
  unsigned long int seq_ctr;

  std::string qtable_init_name;
  bool   training;
  double epsilon;
  double gamma;
  double alpha;
  int    qtable_save_freq;
  bool   minimize;

  bool          read_options(const char* fname , const char*chapter);

  unsigned int  get_opt_action_id(unsigned long int id_x , double* qminvalue=0);
  double        train( unsigned long int id_x , unsigned int id_u , unsigned long int id_x_new , double r , bool terminal );
};

#endif
