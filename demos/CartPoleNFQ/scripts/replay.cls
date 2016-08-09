[Main]
num_episodes = 10
cycles_per_episode = 500
sleep_every_cycle = 10

plant = CartPole
controller = NeuroQ
statistics = GeneralStatistics
observer = DefaultObserver
graphic = CartPoleGraphic

[Input]
input_mode = random
xinit = [-.5 .5] [0 0][-1. 1.] [0 0]

[Output]

[Statistics]
statistics_mode = raw
xwork = [-.7 .7] [][-2.4 2.4][] 
xplus = [-.05 .05] [][-.05 .05][] 
average_over_n_episodes = 1

[Controller]
actions = -10 10
Qnet = replay.tmp.net
working_area = [-1.5 1.5] [][-2.4 2.4][] 

[Plant]
