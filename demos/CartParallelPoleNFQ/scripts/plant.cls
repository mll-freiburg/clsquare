
plant = CartParallelPole
controller = NFQControl
statistics = GeneralStatistics
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

$ config/observer.cls

[Plant]

dynamics = Gomez

[Statistics]

$ config/task.cls
statistics_mode = standardized
average_over_n_episodes = 1

