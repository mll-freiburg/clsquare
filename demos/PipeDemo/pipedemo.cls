[Main]
num_episodes = 1
cycles_per_episode = 1000000

plant = DemoPlant
controller = DemoController
#graphic = UniversalCartPoleGraphic
statistics = GeneralStatistics

sleep_every_cycle = 1000

verbosity = 2
interactive_mode = true

[Input]

input_mode = random
xinit = [4 4] [0 0]

[Output]

[GeneralStatistics]
statistics_mode = standardized
xwork = [-.3 .3] []
xplus = [-.05 .05] []
average_over_n_episodes = 1000
statistics_file = train.stat

[Controller]
#max_action = 2.0 # if action > max_action, controller stops episode
verbosity = 0

[Plant]
verbosity = 1

[Graphic]
active = true
