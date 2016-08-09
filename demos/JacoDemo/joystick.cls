# Simple joystick demo

[Main]
num_episodes         = 1
cycles_per_episode   = 1000000

plant                = JacoBasePlant
controller           = MixControl
output               = DefaultOutput

[Input]
input_mode           = random
xinit                = [0 0][-0.3 -0.3][0.2 0.2][-3.07 -3.07][0.01 0.01][-0.21 -0.21][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][]

[Controller]
controller_0         = CenterControl Hold
action_dim_0         = 1
controller_1         = JoystickControllerEX Joystick
action_dim_1         = 20
controller_2         = MixControlCombine TriggerDifference
action_dim_2         = 1
controller_3         = CenterControl FingerOffset
action_dim_3         = 1
controller_4         = MixControlCombine FingerSubtract
action_dim_4         = 1
# Directly use axes 2, 5 and 1 for cartesian movement, as well as 6 and 7 for wrist alignment.
# The wrist rotation is controller by the difference between axes 3 and 4 (triggers), since they return 1 when even idle, and thus can't be used separately.
# The fingers are controlled by button 1; since its value is 0 or 1, we subtract 0.5 to get two directions.
assignment           = [1 2][1 5][1 1][1 6][1 7][2 0][4 0][4 0][4 0]

[Hold]
mode                 = Static
value                = 0.

[TriggerDifference]
controller_0         = 1
actions_0            = 3
controller_1         = 1
actions_1            = 4
operator             = -

[FingerSubtract]
controller_0         = 1
actions_0            = 8
controller_1         = 3
operator             = -

[FingerOffset]
mode                 = Static
value                = 0.5

[Plant]
delta_t              = 10000
action_mode          = direction
cartesian            = true
pos_min              = -0.2 -0.5 -1    -10 -10 -10 10 10 10
pos_max              =  0.2  0    1     10  10  10 60 60 60
wait_threshold       =  0.05 0.05 0.05 0.2 0.2 0.2
drop                 = -1
reclaim              = false

