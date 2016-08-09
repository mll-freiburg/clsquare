#include "pidcontroller.h"
#include <cstdio>

#include "valueparser.h"

#define BOOLASSERT( __check__ ) \
    if (!(__check__)) { EOUT("The assertion: [" << #__check__ << "] failed."); return false; }

StdPIDController::StdPIDController()
{
    this->error_id      = 0;
    this->action_id     = 0;
    this->anti_windup   = false;
    this->kp            = 0.0;
    this->ki            = 0.0;
    this->kd            = 0.0;
}

StdPIDController::~StdPIDController()
{
    ;
}

bool StdPIDController::get_action (   const double    *observed_state,
                                      double          *action)
{
    this->episode_step_counter += 1;

    double resaction    = 0.0;
    double e            = observed_state[this->error_id];
    double dedt         = (e - this->last_error) / this->dt;
    double tmp_integral = this->error_integral + (e*dt);

    // simple pid control law
    resaction = this->kp * e +
                this->ki * tmp_integral +
                this->kd * dedt;
    // this is an rather weak anti-windup heuristic
    if ( !this->anti_windup || awork.isWithinSet(&resaction,1) )
        this->error_integral = tmp_integral;


    if (resaction<awork.subsets[0][0].min)
        resaction = awork.subsets[0][0].min;
    if (resaction>awork.subsets[0][0].max)
        resaction = awork.subsets[0][0].max;

    this->last_error = e;

    action[this->action_id] = resaction;

    return true;
}

bool StdPIDController::init (         const int       observed_state_dim,
                                      const int       action_dim,
                                      double          deltat,
                                      const char      *fname,
                                      const char      *chapter)
{
    this->odim  = observed_state_dim;
    this->adim  = action_dim;
    this->dt    = deltat;

    // some checks to avoid problems
    BOOLASSERT( this->dt   > 0 );
    BOOLASSERT( this->adim > 0 );
    BOOLASSERT( this->odim > 0 );

    if (fname==0) {
        EOUT("This controller has mandatory options, but no config file given!");
        return false;
    }
    char the_chapter[200];
    if (chapter!=0) sprintf(the_chapter , "%s" , chapter);
    else sprintf(the_chapter , "%s" , "Controller");

    if (!this->read_options(fname,the_chapter)) {
        EOUT("Options could not be read!");
        return false;
    }

    this->episode_counter       = 0;
    this->episode_step_counter  = 0;

    return true;
}

bool StdPIDController::read_options(const char* fname, const char* chapter)
{
    ValueParser vp(fname,chapter);
    char paramstr[1024];
    /** @option xwork : SetDef string **/
    if (vp.get("xwork", paramstr, MAX_STR_LEN)>0) {
        if (!xwork.parseStr(paramstr,odim)) {
            EOUT("Wrong param string xwork: " << paramstr);
            return false;
        }
    } else {
        WOUT( 5 , "No param string xwork given for controller, assuming no restriction.");
    }
    /** @option awork : SetDef string
      * defines the range of the computed action.*/
    if (vp.get("awork", paramstr, MAX_STR_LEN)>0) {
        if (!awork.parseStr(paramstr,1)) {
            EOUT("Wrong param string awork: " << paramstr);
            return false;
        }
    } else {
        EOUT("No param string awork given for controller required a 1dim SetDef!");
        return false;
    }
    BOOLASSERT( awork.numSubsets()      == 1);
    BOOLASSERT( awork.subsets[0].size() == 1);

    vp.get( "error_id"  , error_id  ); BOOLASSERT( error_id  >=0 && error_id  < odim );
    vp.get( "action_id" , action_id ); BOOLASSERT( action_id >=0 && action_id < adim );

    vp.get( "kp" , kp );
    vp.get( "ki" , ki );
    vp.get( "kd" , kd );
    vp.get( "anti_windup" , this->anti_windup );

    return true;
}

void StdPIDController::deinit()
{
    ;
}

void StdPIDController::notify_episode_starts()
{
    this->episode_step_counter   = 0;
    this->episode_counter       += 1;

    this->last_error             = 0;
    this->error_integral         = 0;
}

void StdPIDController::notify_episode_stops(const double* current_observed_state)
{
    ;
}

REGISTER_CONTROLLER( StdPIDController , "A std PID feedback controller for one dimensional actions/errors")
