## NXTpi and My_ExoMy

![ExomyNXT](https://github.com/pebe-espana/NXTpi/blob/ExoMy/My%20ExoMy_NXT/some_documentation/ExomyNXT.jpg)

this is a test version of additions to the ExoMy container - to be updated as I go along.
Happy if you look at this branch, it works (or worked).

Current status:

included driving control improvements by 0xD0M1M0 (see his ExoMy Github):
- joystick_parser_node.py - changes sensitivity of joystick positioning
- rover.py - changes to ackermann algorithm "A", velocity specification in crabbing "C" and point_turn modes "X"
- webinterface improvements to be more stable (gui folder files)

included my Pilot-node additions (see my branch "Additions") with improvements:
- to run preset task scripts of pre-programmed paths (text files inconfig folder) 
- to incorporate the NXT compass sensor data in the motion

under development:
- corrective measures to actual path programmes
- obstacle recognition or tracking along a wall
- more friendly GUI based interaction with ExoMy 

But as it depends on my own hardware including NXT etc, it may not be that directly applicable to other ExoMy fans.

More useful for immediate general use by other ESA-ExoMy enthusiasts is a look at branches of "Additional Features":

ExomyAdditions - Pilot = modules added to add a text command driven pilot, should fit all users of ESA-release ExoMy_Software (ROS melodic, 1.0.3)



