[Main]
num_episodes = 200
cycles_per_episode = 300

plant = MountainCar
controller = TreeController
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
xplus_is_terminal = true
xminus_is_terminal = true
reward_step = 1
reward_xplus = 0
reward_xminus = 1000

[Input]
input_mode = random
xinit = [-0.75 -0.25] [-0.5 0.5]

[Output]

[Statistics]
statistics_mode = standardized
xplus= [0.7 100][]
xwork = [-1.2 100] []
xminus = [-100 -1.2][]
average_over_n_episodes = 1
statistics_file = train.stat

[Controller]
actions = -4.0 4.0
update_freq = 1
gamma = 0.95
epsilon = 0.5
fq_cycles = 10
fileprefix = tree
num_trees = 80
num_tests = 2
n_min = 20
num_threads = 2

[Plant]

[Graphic]
active = true
