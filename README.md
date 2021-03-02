## NXTpi - a lifelong learning journey

![First_outing](https://github.com/pebe-espana/NXTpi/blob/master/First_outing.jpg)
<em> On a first outing </em>


### Branches available

ExoMy = my own version of ESA ExoMy Rover (extended hardware)

ExoMyAddition-Pilot = "Plug-in" addition to facilitate text-based piloting of ExoMy (standard ESA hardware model)

### My own mission
How to inject new life into a LEGO robot built on NXT-brick basis, by upgrading its control and communication with a Rasberry Pi as companion.
Benefits:
- simplicity and flexibility of mechanical construction offered by LEGO components for easy prototyping
- the power of LeJos in controlling motion of a proven basic mobile platform
- adding the processing power of a Raspberry Pi to add modern sensors, communication and multi-tasking possibilities
- exploiting availability of powerful Python libraries on a Raspberry Pi
- allowing long-range two-way communication with mobile platform via WiFi intranet from a GUI on a laptop
- ... and in the process learn what such frameworks as ROS are all about!

### Toolbox and components used

#### Hardware

- NXT-brick with std sensors
- Mindsensor CMPS-Nx, NXTCam
- Raspberry Pi with MakerHawk UPS, Bluetooth and wifi dongle

active robot platform: ExoMy Rover (see https://github.com/esa-prl/ExoMy)

- Raspberry Pi 4 (control with ROS melodic in docker container)
- NXT brick with CMPS-Nx, Ultrasound sensor (connected via Bluetooth to Pi using roslibpy to connect to ROS container)

#### Software

- LeJos
- Java8 and later
- Python3
- ROS
