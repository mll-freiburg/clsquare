[Main]
num_episodes = 2000
cycles_per_episode = 200

plant = CartPole
controller = NFQControl
statistics = GeneralStatistics

call_cmd_freq = 1
call_cmd = scripts/test.bash

#sleep_every_cycle = 10

[Input]
input_mode = random
xinit = [-.5 .5] [0 0][-.5 .5] [0 0]

[Output]

[Statistics]
statistics_mode = standardized
xwork = [-.7 .7] [][-2.4 2.4][] 
xplus = [-.05 .05] [][-.05 .05][] 
average_over_n_episodes = 1
statistics_file = train.stat

[Controller]
controller_1 = balance

[balance]
actions= -10 10

Q.init = Q.init.net
Q_save_path = ./nets

exploration = 0
update_freq=1

verbosity = 1

#mlp:
num_epochs= 300
mlp_verbosity = 2
autoscale_mode = 2
mlp_protfile = nfq.prot
hint2goal = 1

#reward:
xwork = [-.7 .7] [][-2.4 2.4][] 
xplus = [-.05 .05] [][-.05 .05][] 
xplusterminal = [-.0 .0] [][-.0 .0][] 

reward_for_terminal_failure = 1
reward_for_terminal_success = 0
reward_outside_target = 0.01
reward_inside_target = 0.0

[Plant]
