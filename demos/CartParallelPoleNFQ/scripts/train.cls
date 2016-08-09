
[Main]

num_episodes = 10000
cycles_per_episode = 200
sleep_every_cycle = 0
call_cmd_freq = 1
call_cmd = scripts/test.online.bash

$ scripts/plant.cls

statistics_file = log/train.stat

[Output]
output_mode = standard
output_file = log/train.prot

[Input]

input_mode = random
$ config/init.cls

[Controller]

controller_1 = balance

[balance]

$ config/task.cls
$ config/nfq.cls

Q.init = config/Q.init.net
Q_save_path = ./nets
update_freq = 1
verbosity = 1
mlp_verbosity = 2
mlp_protfile = log/nfq.train.prot

