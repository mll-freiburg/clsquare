[Main]
num_episodes = 500
cycles_per_episode = 60

plant = Maze
controller = QTableController
graphic = MazeGraphic
statistics = GeneralStatistics

#optional parameters:
call_cmd_freq = 10
call_cmd = ./test
sleep_every_cycle = 0

verbosity = 10

[Input]
input_mode = from_file
input_file = Maze.init

#input_mode = random
#init = [0 10] [0 10] 

[Output]

[Statistics]
statistics_mode = standardized
xwork = [0 11] [0 11] 
xplus = [7.5 8.5] [1.5 2.5]
average_over_n_episodes = 10
statistics_file = train.stat

[Reward]
xwork = [0 11] [0 11] 
xplus = [7.5 8.5] [1.5 2.5]
xplus_is_terminal=true
reward_step = 1
reward_xplus = 0

[Controller]
alpha = 1.0
epsilon = 0.3
training = true
qtable_name = init.qtable
save_frequency = 10
 
[Graphic]
active = false
#port = 20000 #port for tcp
