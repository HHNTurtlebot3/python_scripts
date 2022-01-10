#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib
import rospy
from sensor_msgs.msg import BatteryState

class Battery_Voltage():
    def __init__(self):
        # rospy.init_node('Battery_Voltage')
        rospy.Subscriber('/battery_state', BatteryState, self.VoltageCallback ,queue_size=10)
        # rospy.spin()

    def VoltageCallback(self, data):
        print('Percent: ' + str(data.percentage))
        print('Voltage: ' + str(data.voltage))

        if(int(data.voltage) <11):
            print('Batterie ist unter 11 V, fahre zurück zur Ladestation')
            return False
        else:
            print('Batteriespannung ist über 11 V')
            return True

def mainVoltage():
    try:
        Battery_Voltage()
        rospy.loginfo('Batterie Check abgeschlossen')
    except rospy.ROSInterruptException:
        rospy.loginfo('exception')

# mainVoltage()