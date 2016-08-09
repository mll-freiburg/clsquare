[Main]
num_episodes = 100
cycles_per_episode = 300

plant = MountainCar
controller = KernelBasedRLController
reward = StandardReward
graphic = MountainCarGraphic
statistics = GeneralStatistics
observer = DefaultObserver

#optional parameters:
call_cmd_freq = 1
call_cmd = ./test
sleep_every_cycle = 10

verbosity = 10

[Reward]
xplus= [0.7 100] []
xminus = [-100 -1.2][]

[Input]
input_mode = random
xinit = [-0.75 -0.25] [-0.5 0.5]

[Output]

[Statistics]
statistics_mode = standardized
xplus= [0.7 100] []
xwork = [-1.2 100] []
xminus = [-100 -1.2][]
average_over_n_episodes = 1
statistics_file = train.stat

[Controller]
threads =1 
epsilon = 0.5
actions = [-4 0 4]
gamma = 0.95
qinit = 0
k = 10

[Plant]

[Graphic]
active = true
