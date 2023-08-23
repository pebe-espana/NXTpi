## Exomy Additions to the ESA ExoMy_Software folder (to be part of docker container)

### Pilot = text based piloting functionality (additional to the existing methods of using webinterface or joystick)

How to implement: 

Add and replace above files in the respective folders of ESA ExoMy_Software, then rerun "sh run_exomy.sh -a"
Remember to add the folders 'action' and 'srv' when adding files, as these activate ROS-actions and ROS-service capabilities.
Also make sure that CMakeLists.txt and package.xml have been replaced.

Notes:

motor_node & robot_node : only changed the rate from 10Hz to 20Hz to get better motion resolution than ESA version

config folder: contains the scripted task file called "keyboard.txt", other names could be used but need to be then declared in "auto_pilot.py"

### Explanations

....in short:

Method 1

- **autojoy_server.py** simply publishes the same /rover_command as the joystick parser node for 'dur' cycles
- command is given by using ros command (requesting mode, vel, deg, dur values ): 'rosservice call /autojoy -- A 2 90 10' , where -- is needed to allow negative values in service call, A is Ackermann mode, 2 is the req.vel, 90 is the req.deg, and 10 is the duration req.dur in 20Hz cycles for which motors are to run

Method 2

- **pilot_exomy.py** is a more advanced version of this functionality using an action service with clients requesting move command
- command is given using one of two possible client versions man_pilot.py, auto_pilot.py
- **man_pilot.py** can be run using 'rosrun exomy man_pilot.py' and it will ask for a string 'A 2 90 10' to be entered directly from keyboard
- **auto_pilot.py** can be run using 'rosrun exomy auto_pilot.py' and it will read the file 'keyboard.txt' and run all the command lines found therein in sequence top to bottom

(for an example of what the config.keyboard.txt results in see the short movie clip in documents folder of this branch)

...more detail: 

- overview which nodes are added to the ESA-Exomy release - see 'ExoMy Addition - Pilot.pdf' in documentation folder 
- to add the files to the docker container:

```
    connect to Exomy with ssh 
    > ssh pi@192.168.nnn.nnn
    
    copy/replace the addition files into the respective ExoMySoftware folders 
    (e.g. transfer to Raspi using VNC connection, or samba, or copy paste in nano editor...)
    
    restart docker container build
    > cd ExoMy_Software/docker
    > sh run_exomy.sh -a
    this starts the ROS container and rosbridge webserver
    
    connect to Exomy with second terminal session using ssh
    > ssh pi@192.168.nnn.nnn
    now find ID of running docker container and connect into it:
    > docker container list
    take note of container ID 
    > docker exec -it [the long container ID] bash
    now you will be in a command line inside the running docker container
    > cd exomy_ws
    > source devel/setup.bash
    now you can run/query etc with ros commands, for example
    > rosnode list
    should show you the running nodes including now /autojoy_server and /pilot-exomy
    
    control rover now from command line
    
    using autojoy:
    >rosservice call /autojoy -- A 30 90 10
    should move the rover by about 13cm forward
    >rosservice call /autojoy -- A -30 90 40 
    should move the rover backwards by about 54cm
    >rosservice call /autojoy -- X 50 0 20
    should do a left turn on the spot by about 90 degrees
    
    using man_pilot:
    >rosrun exomy man-pilot 
    will give you an entry request and expected formatting pattern (2 and 4 character sized patterns):
     -.-...-...-... 
    >A 30  90  10
    
    using auto-pilot reading commands from file keyboard.txt:
    >cd src/exomy/src
    >rosrun exomy auto-pilot
    the rover should now move in a square of 26cm 
    (4 x left turns followed by another square of 4 x right turns)
    after completed run stop the node with CTRL-C to get back to prompt
    
    note that keyboard.txt is in src/exomy/config folder!
    so the following will get you there
    >cd ../config
   
```
### Forthcoming developments planned
 
- add command parsing for direct moves/turns in m and degrees in absolute or relative terms(as part of this add-in branch)
- add sensors and subscriptions to these (hardware dependent, so to be added to ExoMy branch of this repository)
