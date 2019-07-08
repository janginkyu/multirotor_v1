#!/usr/bin/env python

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
        if gpio.input(pinNum) == 0:
            gpioPinEvent(pinNum, False)
        else:
            gpioPinEvent(pinNum, True)
    return cb

def publisher():
    gpio.setmode(gpio.BCM)
    for pinNum in readPin:
        gpio.setup(pinNum, gpio.IN)
        pub[pinNum] = rospy.Publisher('/jik/rpi/gpio/' + str(pinNum), UInt32, queue_size=10)
        gpio.add_event_detect(pinNum, gpio.BOTH, callback=makeCb(pinNum))
    
    rospy.loginfo('GPIO pin init complete.')
    rospy.loginfo('input pin numbers : ')
    for pinNum in readPin:
        print(str(pinNum) + ' ')

    rospy.init_node('gpioRead', anonymous=True)
    rate = rospy.Rate(200)
    while not rospy.is_shutdown():
        for pinNum in readPin:
            if dataRefresh[pinNum]:
                pub[pinNum].publish(dTime[pinNum])
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
