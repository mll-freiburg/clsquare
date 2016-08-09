[Main]

num_episodes = 1
cycles_per_episode = 100000
sleep_every_cycle = 0
verbosity = -1

$ scripts/plant.cls

statistics_file = log/test.stat

[Output]

output_mode = standard
output_file = log/test.prot

[Input]

# single starting state as in Gomez & Miikkulainen
input_mode = random
xinit = [0.07008507 0.07008507] [0 0] [0 0] [0 0] [0 0] [0 0]

[Controller]

controller_1 = balance

[balance]

$ config/task.cls

Q.init = nets/balance.Q.TESTNUM.net
Q_save_path = /dev/null
update_freq = 0
verbosity = -1
mlp_verbosity = -1
mlp_protfile = log/nfq.test.prot

exploration = 0
num_epochs = 300
autoscale_mode = 2
hint2goal = 1

reward_for_terminal_failure = 1
reward_for_terminal_success = 0
reward_outside_target = 0.01
reward_inside_target = 0.0
failure_virtual_loop = true

