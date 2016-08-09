#include "kerneldatatypes.h"
#include "reward.h"

// --- BEGIN --- spec_struct ---------------------------
spec_struct::spec_struct()
{
    this->verbosity             = 0;
    this->interactive_mode      = false;
    this->config_fname          = "";
    this->num_episodes          = 1;
    this->cycles_per_episode    = 2;
    this->call_cmd_freq         = 0;
    this->call_cmd              = "CallCmd";
    this->sleep_every_cycle     = 0;
    this->max_initial_retries   = 100000;
    this->max_external_signal_retries    = 100;
    this->plant_module_name     = "";
    this->controller_module_name= "";
    this->controller_module_name= "";
    this->reward_module_name    = "";
    this->graphic_module_name   = "";
    this->statistics_module_name= "";
    this->reward_module_name    = "";
    this->observer_module_name  = "";
    this->reward_type           = REWARD_INPUT_MEASUREMENT;
    this->do_not_start_in_terminal_states = 1;
}


// --- BEGIN sys_time_struct ----------------------------------------------
sys_time_struct::sys_time_struct(){
    this->reset();
}

void sys_time_struct::reset()
{
    this->cycle_ctr             =  0;
    this->episode_ctr           =  0;
    this->total_num_of_cycles   =  0;
    this->total_time            =  0;
    this->episode_time          =  0;
    this->total_real_time       =  0;
    this->delta_t               = -1;
}

sys_dim_struct::sys_dim_struct()
{
    this->reset();
}

void sys_dim_struct::reset()
{
    this->plant_state_dim       = -1;
    this->measurement_dim       = -1;
    this->observed_state_dim    = -1;
    this->action_dim            = -1;
    this->external_signal_dim   = -1;
}

// --- BEGIN of sys_struct --------------------------------------------------

sys_struct::sys_struct()
{
    // invalidate variables
    this->dim.reset();

    this->prev_state_vars.plant_state          = 0;
    this->prev_state_vars.measurement          = 0;
    this->prev_state_vars.external_signal      = 0;
    this->prev_state_vars.observed_state       = 0;

    this->current_state_vars.plant_state       = 0;
    this->current_state_vars.measurement       = 0;
    this->current_state_vars.external_signal   = 0;
    this->current_state_vars.observed_state    = 0;

    this->next_state_vars.plant_state          = 0;
    this->next_state_vars.measurement          = 0;
    this->next_state_vars.external_signal      = 0;
    this->next_state_vars.observed_state       = 0;

    this->current_action            = 0;
    this->prev_action               = 0;


    // init system time variables
    this->current_time.reset();
}

bool sys_struct::alloc() {
    prev_state_vars.plant_state            = new double[dim.plant_state_dim];
    prev_state_vars.measurement            = new double[dim.measurement_dim];
    prev_state_vars.observed_state         = new double[dim.observed_state_dim];

    current_state_vars.plant_state         = new double[dim.plant_state_dim];
    current_state_vars.measurement         = new double[dim.measurement_dim];
    current_state_vars.observed_state      = new double[dim.observed_state_dim];

    next_state_vars.plant_state            = new double[dim.plant_state_dim];
    next_state_vars.measurement            = new double[dim.measurement_dim];
    next_state_vars.observed_state         = new double[dim.observed_state_dim];

    if (dim.external_signal_dim > 0) {
        current_state_vars.external_signal = new double[dim.external_signal_dim];
        next_state_vars.external_signal    = new double[dim.external_signal_dim];
        prev_state_vars.external_signal    = new double[dim.external_signal_dim];
    }

    current_action              = new double[dim.action_dim];
    prev_action                 = new double[dim.action_dim];

    this->reset_sys_vars();

    return true;
}

#define DELETEIFNOTNULL( __x__ ) if ( (__x__)!=0 ) delete[] (__x__); (__x__)=0;

bool sys_struct::free() {
    DELETEIFNOTNULL( prev_state_vars.plant_state );
    DELETEIFNOTNULL( prev_state_vars.measurement );
    DELETEIFNOTNULL( prev_state_vars.observed_state );
    DELETEIFNOTNULL( prev_state_vars.external_signal );

    DELETEIFNOTNULL( current_state_vars.plant_state );
    DELETEIFNOTNULL( current_state_vars.measurement );
    DELETEIFNOTNULL( current_state_vars.observed_state );
    DELETEIFNOTNULL( current_state_vars.external_signal );

    DELETEIFNOTNULL( next_state_vars.plant_state );
    DELETEIFNOTNULL( next_state_vars.measurement );
    DELETEIFNOTNULL( next_state_vars.observed_state );
    DELETEIFNOTNULL( next_state_vars.external_signal );

    DELETEIFNOTNULL( this->current_action );
    DELETEIFNOTNULL( this->prev_action );

    return true;
}

void sys_struct::reset_sys_vars()
{
    for (int i=0; i<dim.plant_state_dim;i++) {
        prev_state_vars.plant_state[i]         = 0.0;
        current_state_vars.plant_state[i]      = 0.0;
        next_state_vars.plant_state[i]         = 0.0;
    }

    for (int i=0; i<dim.measurement_dim;i++) {
        prev_state_vars.measurement[i]         = 0.0;
        current_state_vars.measurement[i]      = 0.0;
        next_state_vars.measurement[i]         = 0.0;
    }

    for (int i=0; i<dim.observed_state_dim;i++) {
        prev_state_vars.observed_state[i]      = 0.0;
        current_state_vars.observed_state[i]   = 0.0;
        next_state_vars.observed_state[i]      = 0.0;
    }

    for (int i=0; i<dim.external_signal_dim; i++) {
        prev_state_vars.external_signal[i]     = 0.0;
        current_state_vars.external_signal[i]  = 0.0;
        next_state_vars.external_signal[i]     = 0.0;
    }

    for (int i=0; i<dim.action_dim;i++)  {
        current_action[i]           = 0.0;
        prev_action[i]              = 0.0;
    }
}
// --- END --- sys_struct ---
