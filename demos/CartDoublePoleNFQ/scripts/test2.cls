# test for large starting region

[Main]
num_episodes = 10
cycles_per_episode = 500

plant = CartDoublePole
controller = NFQControl
statistics = GeneralStatistics
graphic = DefaultGraphic

[Input]
input_mode = random
$ config/init.cls

[Output]

[Statistics]
statistics_mode = raw
$ config/target.cls
statistics_file = test.tmp.stat
average_over_n_episodes = -1

[Controller]
controller_1 = balance

[balance]
$ config/target.cls
$ config/actions.cls

Q.init = Q.balance.tmp.net
exploration = 0
update_freq = 0

[Plant]

