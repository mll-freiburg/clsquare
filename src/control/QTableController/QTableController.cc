#include "QTableController.h"
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include "valueparser.h"

QTableController::QTableController()
{
    this->qtable = 0;

    qtable_init_name = "qtable_init.txt";
    epsilon  = 0.0;
    alpha    = 1.0;
    gamma    = 1.0;
    training = true;
    minimize = true;
    this->qtable_save_freq = 100;
}

QTableController::~QTableController()
{
    if (this->qtable!=0) delete this->qtable;
}

bool QTableController::get_action(const double* observed_state, double* action)
{
    unsigned int id_u = 0;

    if (epsilon==0 || drand48() > epsilon)
        id_u = this->get_opt_action_id( qtable->get_state_id(observed_state) );
    else
        id_u = (int) ( drand48() * qtable->number_of_actions() );

    qtable->copy_action( id_u , action );

    return true;
}

bool QTableController::init(const int observation_dim, const int action_dim, double deltat, const char* fname, const char* chapter)
{
  qtable = new QTable( observation_dim , action_dim );

  if (!this->read_options( fname , chapter )) {
      EOUT("Error on reading options.");
      return false;
  }

  if (!qtable->load( qtable_init_name.c_str() )) {
      EOUT("Can not load initial qtable");
      return false;
  }

  if (!training && epsilon > 0) {
      WOUT(0,"Training switched off but epsilon greedy exploration not 0: " << this->epsilon);
  }

  seq_ctr = 1;

  return true;
}

bool QTableController::read_options(const char* fname , const char*chapter)
{
    if(fname == 0)
        return true;

    char chapterbuf[200];
    if (chapter==0) sprintf(chapterbuf,"Controller");
    else sprintf(chapterbuf,"%s", chapter);

    ValueParser vp(fname,chapterbuf);

    char strbuf[100];
    if(vp.get("qtable_name" , strbuf, 100)>0) {
        qtable_init_name = strbuf;
    } else {
        EOUT("No parameter [qtable_name =]<filename> given.");
        return false;
    }

    vp.get("epsilon"    , epsilon );
    vp.get("gamma"      , gamma);
    vp.get("alpha"      , alpha);
    vp.get("training"   , training);
    vp.get("minimize"   , minimize );
  vp.get("save_frequency", this->qtable_save_freq, 100); 

    return true;
}

void QTableController::deinit()
{
    if (this->training) {
        if (this->qtable!=0) this->qtable->save( "qtable_trained.txt" , true );
    }
    if (this->qtable!=0) delete this->qtable;
    this->qtable=0;
}


void QTableController::notify_transition(const double* observed_state,
                                         const double* action,
                                         const double* next_observed_state,
                                         const double reward,
                                         const bool is_terminal_state,
                                         const double terminal_reward)
{
    if (!this->training) return;

    unsigned long int id_x      = qtable->get_state_id( observed_state );
    unsigned long int id_xnew   = qtable->get_state_id( next_observed_state );
    unsigned int      id_u      = qtable->get_action_id( action );

    this->train( id_x , id_u , id_xnew , reward , is_terminal_state );
}

void QTableController::notify_episode_stops(const double* current_observed_state)
{
    this->seq_ctr+=1;

    if (this->training && this->seq_ctr%this->qtable_save_freq==0) {
       IOUT("Saving qtable ... ");
       this->qtable->save( "qtable_trained.txt" , true );
    }
}


double  QTableController::train( unsigned long int id_x , unsigned int id_u , unsigned long int id_x_new , double r , bool terminal )
{
    double be   = 0;
    double qnew = 0;
    double qist = 0;

    qist = qtable->get_value( id_x , id_u );

    if (terminal) {
        qnew = r;
    } else {
        double qminxnew=0;
        this->get_opt_action_id(id_x_new , &qminxnew);
        qnew = gamma*qminxnew + r;
    }

    be = qnew - qist;

    qnew = (1-alpha) * qist + alpha * qnew;

    qtable->set_value( id_x , id_u , qnew );

    return be;
}

unsigned int QTableController::get_opt_action_id(unsigned long int id_x, double* qoptvalue)
{
    double qopt = 0;
    std::vector<unsigned int> id_u_opt;

    for (unsigned int id_u=0; id_u<this->qtable->number_of_actions(); id_u++) {
        double q = this->qtable->get_value( id_x , id_u );
        if ( id_u==0 || (this->minimize ? q < qopt : q > qopt) ) {
            qopt = q;
            id_u_opt.clear();
            id_u_opt.push_back( id_u );
        } else if ( q == qopt ) {
            id_u_opt.push_back( id_u );
        }
    }

    assert(id_u_opt.size()>0);

    if (qoptvalue!=0) *qoptvalue=qopt;

    if (id_u_opt.size() == 1) return id_u_opt[0];

    int rnd_id_id = (int) (drand48() * (int)id_u_opt.size());

    return id_u_opt[rnd_id_id];
}


REGISTER_CONTROLLER( QTableController , "A learning RL controller, based on Q-learning with a table.");

