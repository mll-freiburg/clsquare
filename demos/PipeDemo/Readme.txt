Demo to show how clsquare can take commands via a pipe mechanism.
You can use the bash scripts send_plandcmd.bash, pause_clsquare.bash,
stop_clsquare.bash or continue_clsquare.bash while clsquare is running.

'stop' will terminate clsquare immediately
'pause' will make a pause
'plant_cmd <something>' will send the command something to plant

Alternatively, you can write into the pipe /tmp/pipe2cls directly
(see the scripts).


Usage:

CLSquare pipedemo.cls

Then, from another terminal call one of the bash scripts or write
directly into the pipe.

