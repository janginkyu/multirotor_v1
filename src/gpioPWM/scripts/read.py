#!/usr/bin/env python

from time import sleep
from datetime import datetime
import RPi.GPIO as gpio

# ros
import rospy
from std_msgs.msg import UInt32

readPin = [14]
riseTime = {}
isUp = {}
dTime = {}
dataRefresh = {}
pub = {}

def gpioPinEvent(pinNum, is_rise=False):
    if is_rise:
        if not isUp[pinNum]:
            riseTime[pinNum] = datetime.utcnow().microsecond
            isUp[pinNum] = True
    else:
        if isUp[pinNum]:
            dTime[pinNum] = datetime.utcnow().microsecond - riseTime[pinNum]
            if dTime[pinNum] < 0:
                dTime[pinNum] += 1000000
            isUp[pinNum] = False
            dataRefresh[pinNum] = True
        else:
            pass

def makeCb(pinNum):
    def cb(channel):
        #rospy.loginfo('cb ' + str(gpio.input(pinNum)))
        if gpio.input(pinNum) == 0: 
            gpioPinEvent(pinNum, False)
        else:
            gpioPinEvent(pinNum, True)
    return cb

def publisher():
    rospy.init_node('gpioRead', anonymous=True)

    gpio.setmode(gpio.BCM)
    for pinNum in readPin:
        gpio.setup(pinNum, gpio.IN)
        pub[pinNum] = rospy.Publisher('/jik/rpi/gpio/' + str(pinNum), UInt32, queue_size=10)
        gpio.add_event_detect(pinNum, gpio.BOTH, callback=makeCb(pinNum))
        dataRefresh[pinNum] = False
        isUp[pinNum] = False

    rospy.loginfo('GPIO pin init complete.')
    rospy.loginfo('input pin numbers : ')
    for pinNum in readPin:
        print(str(pinNum) + ' ')

    rate = rospy.Rate(40)
    while not rospy.is_shutdown():
        for pinNum in readPin:
            if dataRefresh[pinNum]:
                pub[pinNum].publish(dTime[pinNum])
                dataRefresh[pinNum] = False
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
