[Main]
num_episodes = 10
cycles_per_episode = 500

plant = CartPole
controller = StateController
graphic = CartPoleGraphic
statistics = GeneralStatistics

sleep_every_cycle = 20

verbosity = 1

[Input]
input_mode = random
xinit = [-.1 .1] [0 0] [-.5 .5] [0 0] 

[Output]

[GeneralStatistics]
statistics_mode = standardized
xwork = [-.3 .3] [] [-2.4 2.4] []
xplus = [-.05 .05] [] [-.2 .2] []
average_over_n_episodes = 1000
statistics_file = train.stat

[Controller]
xwork = [] [][][]
gain = 30.61  7.77  .45  1.72 
 
[Plant]
angle_max = 2.0
position_max = 1.0

[Graphic]
active = true
