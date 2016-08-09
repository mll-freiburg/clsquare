# test for replay

[Main]
num_episodes = 20
cycles_per_episode = 500
sleep_every_cycle = 20

plant = CartDoublePole
controller = NeuroQ
statistics = GeneralStatistics

[Input]
input_mode = random
$ log/config/init.cls

[Output]
output_file = replay.tmp.prot
output_mode = standard

[Statistics]
$ log/config/target.cls
stop_in_xplus = false

[Controller]
nfqmode = true
$ log/config/target.cls
$ log/config/actions.cls
Qnet = replay.tmp.net
stop_in_xplus = false

[Plant]

