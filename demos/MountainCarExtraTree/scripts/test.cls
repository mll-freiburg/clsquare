[Main]
num_episodes = 1000
cycles_per_episode = 300

plant = MountainCar
controller = TreeController
reward = StandardReward
graphic = MountainCarGraphic
statistics = GeneralStatistics
observer = DefaultObserver

#optional parameters:
#sleep_every_cycle = 30

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
noheader = true
xwork = [-1.2 100] [] 
xplus = [0.7 100] []
average_over_n_episodes = 1000
statistics_file = test.stat

[Controller]
actions = -4.0 4.0
# for testing previously trained controller update_freq and epsilon must be 0
update_freq = 0
epsilon = 0.0
# prefix of previously trained ExtraTree
fileprefix = tree

[Plant]

[Graphic]
active = false
