[Main]

num_episodes = 10
cycles_per_episode = 500
sleep_every_cycle = 20
verbosity = -1

plant = CartParallelPole
controller = NeuroQ
observer = ChainObserver

[Observer]

observer_0 = HistoryObserver LastState
observer_1 = ArithmeticObserver StateDifference
observer_2 = ShuffleObserver Blind

[LastState]

memory = 1
assignment = 0 2 4

[StateDifference]

operation_0 = $0 - $6
operation_1 = $2 - $7
operation_2 = $4 - $8

[Blind]

assignment = 0 9 2 10 4 11

[Plant]

dynamics = Gomez

[Input]

# single starting state as in Gomez & Miikkulainen
input_mode = random
xinit = [-0.07 0.07] [0 0] [-0.01 0.01] [0 0] [-0.1 0.1] [0 0]

[Controller]

actions      = -10 0 10
working_area = [-.7 .7] [] [-.7 .7] [] [-2.4 2.4] [] 
Qnet         = replay.tmp.net

