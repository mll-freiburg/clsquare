# Simple joystick demo

[Main]
num_episodes         = 1
cycles_per_episode   = 1000000

plant                = JacoBasePlant
controller           = MixControl

interactive_mode = true

[Input]
input_mode           = random
xinit                = [0 0][-0.3 -0.3][0.2 0.2][-3.07 -3.07][0.01 0.01][-0.21 -0.21][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][]

[Controller]
controller_0      = CenterControl Center
controller_1      = CenterControl Left
controller_2      = CenterControl Right
controller_3      = CenterControl Top
controller_4         = MixControlPipe Select
assignment           = [4 0][4 1][4 2][4 3][4 4][4 5][4 6][4 7][4 8]

[Select]
controller_0 = 0
command_0 = controller_cmd center
controller_1 = 1
command_1 = controller_cmd left
controller_2 = 2
command_2 = controller_cmd right
controller_3 = 3
command_3 = controller_cmd top

[Center]
mode                 = Static
value                = 0 -0.35 0.15 -3.07 0.01 -0.21 20 20 20

[Top]
mode                 = Static
value                = 0 -0.25 0.3 -3.07 0.01 -0.21 5 5 5

[Right]
mode                 = Static
value                = 0.1 -0.3 0.15 -3.07 0.00 -0.22 10 20 30

[Left]
mode                 = Static
value                = -0.1 -0.3 0.15 -3.07 0.02 -0.20 20 30 10

[Plant]
delta_t              = 10000
action_mode          = position
cartesian            = true
pos_min              = -0.2 -0.5 -1    -10 -10 -10 10 10 10
pos_max              =  0.2  0    1     10  10  10 60 60 60
wait_threshold       =  0.05 0.05 0.05 0.2 0.2 0.2
drop                 = -1
reclaim              = false

