[Main]
num_episodes = 1000
cycles_per_episode = 300

plant = MountainCar
controller = NeuroQ
reward = StandardReward
graphic = MountainCarGraphic
statistics = GeneralStatistics

#optional parameters:
sleep_every_cycle = 30

verbosity = 0

[Input]
input_mode = from_file
input_file = evaluation.init

[Output]

[Reward]
xplus= [0.7 100] []
xminus = [-100 -1.2][]

[Statistics]
statistics_mode = standardized
xwork = [-1.2 100] [] 
xplus = [0.7 100] []
average_over_n_episodes = 1
statistics_file = test.stat

[Controller]
nfqmode = true
actions= -4 4
Qnet = Q.net
stop_in_xplus = true
xwork = [-1.5 1.5] [][-2.4 2.4][] 


[Plant]

[Graphic]
active = true
