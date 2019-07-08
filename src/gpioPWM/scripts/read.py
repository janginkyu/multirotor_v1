#!/usr/bin/env python

from time import sleep
from datetime import datetime
import RPi.GPIO as gpio

# ros
import rospy
from std_msgs.msg import UInt32

readPin = [18]
riseTime = {}
isUp = {}
dTime = {}
dataRefresh = {}
pub = {}
dTimeSum = {}
dTimeNum = {}
prevDat = {}

def gpioPinEvent(pinNum, is_rise=False):
    if is_rise:
        if not isUp[pinNum]:
            riseTime[pinNum] = datetime.utcnow().microsecond
            isUp[pinNum] = True
    else:
        if isUp[pinNum]:
            temp = datetime.utcnow().microsecond - riseTime[pinNum]
            if temp < 0:
                temp += 1000000
            if (temp > 900) and (temp < 2100):
                if (prevDat[pinNum] == -1) or (abs(prevDat[pinNum] - temp) < 100):
                    dTimeSum[pinNum] += temp
                    dTimeNum[pinNum] += 1
                    dataRefresh[pinNum] = True
                prevDat[pinNum] = temp
            isUp[pinNum] = False
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
        dTimeSum[pinNum] = 0
        dTimeNum[pinNum] = 0
        dTime[pinNum] = 0
        prevDat[pinNum] = -1

    rospy.loginfo('GPIO pin init complete.')
    rospy.loginfo('input pin numbers : ')
    for pinNum in readPin:
        print(str(pinNum) + ' ')

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        for pinNum in readPin:
            if dataRefresh[pinNum]:
                dTime[pinNum] = dTimeSum[pinNum] / dTimeNum[pinNum]
                pub[pinNum].publish(dTime[pinNum])
                dataRefresh[pinNum] = False
                dTimeSum[pinNum] = 0
                dTimeNum[pinNum] = 0
                rospy.loginfo((' ' * ((dTime[pinNum] - 800) / 40)) + '*')
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
