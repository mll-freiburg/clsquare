[Main]
num_episodes = 2000
cycles_per_episode = 200

plant = CartDoublePole
controller = NFQControl
statistics = GeneralStatistics

call_cmd_freq = 1
call_cmd = scripts/test.bash

graphic = DefaultGraphic
sleep_every_cycle = 0

[Input]
input_mode = random
$ config/init.cls

[Output]

[Statistics]
statistics_mode = standardized
$ config/target.cls
statistics_file = train.stat
average_over_n_episodes = 1

[Controller]
controller_1 = balance

[balance]
$ config/target.cls
$ config/actions.cls

Q.init = Q.init.net
Q_save_path = ./nets
save_net_freq = 1

exploration = 0
update_freq = 1

verbosity = 1

#mlp:
num_epochs = 300
mlp_verbosity = 2
autoscale_mode = 2
hint2goal = 1

#reward:
reward_for_terminal_failure = 1
reward_for_terminal_success = 0
reward_outside_target = 0.01
reward_inside_target = 0.0

[Plant]

