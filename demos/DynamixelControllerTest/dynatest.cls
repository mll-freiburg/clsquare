[Main]

num_episodes = 1
cycles_per_episode = 200000

plant = DynamixelEncoderTest
controller = CenterControl
tatistics = Statistics

[Input]

input_mode = random
xinit = [] [] [] []

[Statistics]

statistics_mode = standardized
xwork = [][][][]
xplus = [][][][]
average_over_n_episodes = 1
statistics_file = ./test.stat

[Controller]

# use a simple bang-bang controller; good for measuring command latencies
mode = BangBang

# for prettier movement one could use a proportional controller, which slows down at 150 encoders from the target
#mode = Proportional
#scaling = 150

# negative actions if above target value
multi = -1 -1 -1 -1

# aim for center position
targets = 512 512 512 512

[Plant]

# cycle duration
delta_t = 10

############ robot types; choose one ###############

# standard single servo at id 1
servos = 1
ids    = 1

# GlotzBot: horizontal, left cam, right cam, vertical
#servos = 4
#ids    = 1 3 2 4

# Dynabutler: single joints 9-7 from tip upward
#             shoulder joint 2 is too weak for the strain; double joints 3/5 and 4/6 cannot be used this way!
#servos = 3
#ids    = 9 8 7

# Bioloid: limit to feet, safest to use
#servos = 2
#ids    = 17 18

############ end robot types #######################

# initial position
start      = 700 700 700 700

# speed at controller action == 1
scaling    = 100 100 100 100
# for Morpheus firmware the maximum speed is 10
#scaling    = 5 5 5 5

# maximum torque; 100 is quite safe even with RX-64
# ignored by Morpheus!
torque     = 150 150 150 150

# true for FTD2xx, false for FTDI
# FTD2xx is faster, but not available on all client architectures
use_ftd2xx = true

# true for custom Morpheus firmware, false for original Robotis firmware
# Morpheus supports synchronous reading, but works only on some reflashed AX-12 servos
#custom_firmware = true

