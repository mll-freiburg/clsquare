[Main]
num_episodes = 1000
cycles_per_episode = 300

plant = MountainCar
controller = KernelBasedRLController
reward = StandardReward
graphic = MountainCarGraphic
statistics = GeneralStatistics
observer = DefaultObserver

#optional parameters:
#sleep_every_cycle = 30

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
average_over_n_episodes = -1
statistics_file = test.stat

[Controller]
epsilon = 0
actions = [-4 0 4]
learn = false
load_kq = tmp.approx

[Plant]

[Graphic]
active = false
