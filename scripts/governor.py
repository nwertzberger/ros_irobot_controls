#!/usr/bin/env python

import rospy
from ros_irobot_create.msg import SensorPacket
from ros_irobot_create.srv import Leds

class Governor():
    def checkPower(self, powerLevel, voltageLevel):
        """
        This function is responsible for checking to see if:
            * remaining power is at a critical level (arbitrarily chosen at 500 mAH)
            * if voltage dips to 10% above of what our power converter needs (6.6V)
        If either of these cases occurs, it constitutes an emergency and ROS needs
        to be stopped.
        """
        if powerLevel < , voltageLevel
    def checkButtons(self, playButton, advanceButton):

    def start(self):
        rospy.init_node("governor")
        rospy.loginfo("Starting Governor")

if __name__ == '__main__':
    try:
        gov = Governor()
        gov.start()
    except rospy.ROSInterruptException:
        pass

