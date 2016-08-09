[Main]
num_episodes = 10
cycles_per_episode = 500

plant = CartPole
controller = NFQControl
statistics = GeneralStatistics

#sleep_every_cycle = 5

[Input]
input_mode = random
xinit = [-.5 .5] [0 0][-1. 1.] [0 0]

[Output]

[Statistics]
statistics_mode = raw
xwork = [-.7 .7] [][-2.4 2.4][] 
xplus = [-.05 .05] [][-.05 .05][] 
statistics_file = test.tmp.stat
average_over_n_episodes = -1

[Controller]

controller_1 = balance

[balance]

actions= -10 10
Q.init = Q.balance.tmp.net
xwork = [-1.5 1.5] [][-2.4 2.4][] 
update_freq=0

[Plant]
