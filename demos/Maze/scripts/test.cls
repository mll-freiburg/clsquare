[Main]
num_episodes = 14
cycles_per_episode = 20

plant = Maze
controller = QTableController
graphic = MazeGraphic
statistics = GeneralStatistics
observer = DefaultObserver

#optional parameters:

sleep_every_cycle = 80

[Input]
input_mode = from_file
input_file = Maze.init

[Output]

[Statistics]
statistics_mode = standardized
noheader = true
xwork = [0 11] [0 11] 
xplus = [7.5 8.5] [1.5 2.5]
average_over_n_episodes = 14
statistics_file = test.stat

[Reward]
xwork = [0 11] [0 11] 
xplus = [7.5 8.5] [1.5 2.5]
xplus_is_terminal=true

[Controller]
epsilon = 0
training = false
qtable_name = qtable_trained.txt
 
[MazeGraphic]
active = true
#port = 20000 #port for tcp
