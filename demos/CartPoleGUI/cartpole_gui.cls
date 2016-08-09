[Main]
num_episodes         = 1
cycles_per_episode   = 10000
sleep_every_cycle    = 10
plant                = CartPole
controller           = StandardController
verbosity            = 1

[Input]
input_mode           = random
xinit                = [0 0] [0 0] [0 0] [0 0] [0 0] [0 0]

[Output]
output_mode          = standard
output_file          = out.log

[Controller]
controller_mode      = 2
gui_request_string   = SLIDER Action -1.0 1.0 0 100
 
[Plant]

[Graphic]
