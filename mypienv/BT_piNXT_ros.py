#!/usr/bin/env python3

# ---- BT_piNXT_ros ----
# this code is to set up a bluetooth connection to NXT
# interact with NXT to obtain Robot sensor data
# and publish this data to the ROS system via websocket localhost 9090

# interacts with NXT_pi_ROS on NXT
# change log:
# - v1 - works but issue that acquired data ages until being published
# - v2 - (14-3-2021) publish data as soon as acquired

import time
import roslibpy
import sys
import bluetooth

# NXT specification
target_name ="NXT2"
target_address = None
port = 1

# ROS system specification
ROS_host = 'localhost'
ROS_port = 9090

#------ step 1 ----
no_NXT = True
while no_NXT :
    print('step 1:')
    print(' looking for NXT in order to connect')
    nearby_devices = bluetooth.discover_devices()

    print(' resolving names of nearby devices...')
    for bdaddr in nearby_devices:
        if target_name == bluetooth.lookup_name( bdaddr ):
            target_address = bdaddr
            break

    if target_address is not None:
        print (' found target bluetooth device with address ', target_address)
        no_NXT = False
    else:
        print (' could not find target bluetooth device nearby')

# gets here only if a target address has been found
print (' connecting via Bluetooth...')
sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
sock.connect((target_address, port))

#------ step 2 -----

print('step 2:')
print(' connecting to ROS ')

client = roslibpy.Ros(ROS_host, ROS_port)
client.run()
print('Is ROS connected?',client.is_connected)

print(' let ROS know that NXT is connected ')
talker = roslibpy.Topic(client,'/NXTready','std_msgs/String')

if client.is_connected == True :
    talker.publish(roslibpy.Message({'data':'NXT and ROS are connected'}))
else :
    talker.publish(roslibpy.Message({'data':'Error: ROS is NOT connected'}))
    print('Error - ROS or NXT are not connected')
    sys.exit()

#---- step 3 ----

print('step 3:')
print(' data exchange ROS - NXT ')
# note for the first two times there is a verbose output

firstloops = True
verbose = 2

battery = roslibpy.Topic(client,'/NXTbattery','std_msgs/Float32')
compass = roslibpy.Topic(client,'/NXTcmps','std_msgs/Float32')
sonar   = roslibpy.Topic(client,'/NXTsonar','std_msgs/Int32')


while client.is_connected:

    if firstloops:
        print(' send one chr (ascii code 0-9)')
    # for k in range(0,4):
    k=1
    data = chr(48+k)
    l0 = len(data) & 0xFF
    l1 = (len(data) >>8) & 0xFF
    if firstloops:
        print (" sending "+ data)
    d = chr(l0) + chr(l1) + data
    sock.send(d)

    if firstloops:
        print(' wait for reply sequence from NXT')

    # first data block = battery voltage
    # NXT
    data = sock.recv(2)
    if firstloops:
        print(data)
    plen = int.from_bytes(data,"little")  # last(2nd) byte is most significant
    if firstloops:
        print(plen)
    data = sock.recv(plen)
    if firstloops:
        print(data)
    # ROS
    datastr1 = data.decode("utf-8")
    if firstloops:
        print(datastr1)
    # parse float32 from string, changing mV to V
    datafloat1 = float(datastr1)/1000.0
    if firstloops:
        timestr = str(time.time())
        print(' battery received ' + timestr)
    # publish data in ROS
    battery.publish(roslibpy.Message({'data': datafloat1}))
    if firstloops:
        print(' battery published')

    # compass
    # NXT
    data = sock.recv(2)
    if firstloops:
        print(data)
    plen = int.from_bytes(data,"little")
    if firstloops:
        print(plen)
    data = sock.recv(plen)
    if firstloops:
        print(data)
    # ROS
    datastr2 = data.decode("utf-8")
    if firstloops:
        print(datastr2)
    # parse float32 from string, changing 3600 to 360
    datafloat2 = float(datastr2)/10.0
    if firstloops:
        timestr = str(time.time())
        print(' compass received ' + timestr)

    # publish data in ROS
    compass.publish(roslibpy.Message({'data': datafloat2}))
    if firstloops:
        print(' compass published')

    # sonar distance
    # NXT
    data = sock.recv(2)
    plen = int.from_bytes(data,"little")
    data = sock.recv(plen)
    if firstloops:
        print(data)
    # ROS
    datastr3 = data.decode("utf-8")
    if firstloops:
        print(datastr3)
    # parse int32 from string
    datafloat3 = float(datastr3)
    dataint = int(datafloat3)
    if firstloops:
        timestr = str(time.time())
        print(' sonar received ' + timestr)

    # publish data in ROS
    sonar.publish(roslibpy.Message({'data': dataint}))
    if firstloops:
        print(' sonar published')

    # now inform terminal that all values read
    timestr = str(time.time())
    print(timestr + " " + datastr1 + " " + datastr2 + " " + datastr3)

    verbose = verbose - 1
    if verbose<0 :
        verbose = 0
        firstloops = False

# 1 second wait => observe about 1.78 sec between publications
# 0.5 sec  wait => observe about 1.3 sec betwen publications

    time.sleep(0.1)

# last step - disconnect all
# ROS

talker.unadvertise()
battery.unadvertise()
compass.unadvertise()
sonar.unadvertise()

client.terminate()

print ('Closing Bluetooth connection...')
sock.close()
print ('Bluetooth connection closed.')

