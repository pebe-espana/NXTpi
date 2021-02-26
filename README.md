## Exomy Additions to the ESA ExoMy_Software folder (to be part of docker container)

### Pilot = text based piloting functionality (additional to the existing methods of using webinterface or joystick)

How to implement: Add and replace files in the respective folders of ESA ExoMy_Software, then rerun "sh run_exomy.sh -a"

Notes:

motor_node & robot_node : only changed the rate from 10Hz to 20Hz to get better motion resolution than ESA version

config folder: contains the scripted task file called "keyboard.txt", other namwes could be used but needs to be then declared in "auto_pilot.py"

### Explanations

in short:

- autojoy_server.py simply publishes the same /rover_command as the joystick parser node for 'dur' cycles
- command is given by using ros command (requesting mode, vel, deg, dur values ): 'rosservice call /autojoy -- A 2 90 10' , where -- is needed to allow negative values in service call, A is Ackermann mode, 2 is the req.vel, 90 is the req.deg, and 10 is the duration req.dur in 20Hz cycles for which motors are to run

- pilot_exomy.py is amore advanced version of this functionality using action service
- command is given using two client versions man_pilot.py, auto_pilot.py
- man_pilot.py can be run using 'rosrun exomy man_pilot.py' and it will ask for a string 'A 2 90 10' to be entered from keyboard
- auto_pilot.py will read the file 'keyboard.txt' and run all the command lines found therein in sequence top to bottom





