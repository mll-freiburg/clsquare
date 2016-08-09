[Main]
num_episodes = 100
cycles_per_episode = 300

plant = MountainCar
controller = NFQControl

reward = StandardReward
graphic = MountainCarGraphic
statistics = GeneralStatistics

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
xplus= [0.7 100][]
xwork = [-1.2 100] []
xminus = [-100 -1.2][]
average_over_n_episodes = 1
statistics_file = train.stat


[Controller]

controller_1 = swingup

[swingup]

actions= -4 4

Q.init = Q.init.net
Q_save_path = ./nets
net_save_freq = 0
reread_xux = true
update_freq=1
exploration = .5

verbosity = 1

#mlp:
num_epochs= 300
mlp_verbosity = 2
autoscale_mode = 2
mlp_protfile = nfq.prot
#hint2goal = 1

#reward:

xplus= [0.7 100] []
xwork = [-1.2 100] []
xplusterminal =  [0.7 100] []

reward_for_terminal_failure = 1
reward_for_terminal_success = 0
reward_outside_target = 0.01
reward_inside_target = 0.0


[Plant]

[Graphic]
active = true
