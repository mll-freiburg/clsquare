
[Main]

plant      = MLLCartPole
controller = NFQControl
statistics = GeneralStatistics
reward     = CartPoleReward

verbosity          = 0
num_episodes       = 20
cycles_per_episode = 2000
sleep_every_cycle  = 0

[Input]
input_mode = random
xinit = ([ -3.14 -2.8][0 0][-.2 .2][0 0], [2.8 3.14][0 0][-.2 .2][0 0])

[Output]

[Statistics]
statistics_mode = standardized
xplus = [-0.1 0.1][][-.15 .15][] 
xwork = [][][-.25 .25][]
average_over_n_episodes = -1
statistics_file = test.tmp.stat

[Reward]
Udiag = 1
quadratic_costs = false

[Controller]
controller_1 = swingup

[swingup]
update_freq = 0
#core:
actions = -1. 0 1.
Q.init = Q.swingup.tmp.net
multi_steps_max = 1
use_action_history = 1
#reward:
xwork = [][][-.25 .25][][]

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
