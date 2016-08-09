
[Main]
plant      =  MLLCartPole
controller =  NFQControl
statistics = GeneralStatistics

verbosity          = 0
num_episodes       = 1000
cycles_per_episode = 800
sleep_every_cycle  = 0

call_cmd_freq = 1
call_cmd = scripts/test.bash

[Input]
input_mode = random
xinit      = ([ -3.14 -2.8][0 0][-.249 .249][0 0], [2.8 3.14][0 0][-.249 .249][0 0])

[Output]

[Statistics]
statistics_mode = standardized
xplus = [-0.1 0.1][][-.1 .1][] 
xwork = [][][-.25 .25][]
average_over_n_episodes = 1
statistics_file = train.stat

[Controller]
controller_1 = swingup

[swingup]
# special parameters
hint2goal = 1
multi_steps_max = 5
use_action_history = 1
force_zeropatterns = true
#do_qmin_heuristic = true
#failure_virtual_loop = true
reward_mode = minimum_time
xplusterminal = [0 0][0 0][0 0][0 0][]
#xtanh = [-.2 .2][][-.25 .25][][]
xplus = [-.1 .1][][-.15 .15][][]

# standard parameters
update_freq = 1
actions = -1. 0 1.
Q_save_path = ./nets
verbosity = 1
Q.init = Q.init.net
#mlp:
num_epochs= 300
mlp_verbosity = 1
do_autoscale = 2
mlp_protfile = nfq.prot
#exploration = 0.1
#exploration_freq = 2

#reward:
xwork = [][][-.25 .25][][]
reward_for_terminal_failure = 1
reward_for_terminal_success = 0
reward_outside_target = 0.01
reward_inside_target = 0.0

[Plant]
num_integration_steps   = 10
delta_t                 = 0.01
delay                   = 0.0
mc                      = 1.05956
mp                      = 0.461056
lp                      = 0.264746
r                       = 0.0185
k2                      = 0.157212
k3                      = 0.065364
Jm                      = 0.00997089
k3_0                    = 0.0315336
stop_if_turnover = 2
