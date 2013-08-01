#!/usr/bin/env python

import rospy
import subprocess
from ros_irobot_create.msg import SensorPacket
from ros_irobot_create.srv import Leds

# Global constants
MIN_CHARGE_LEVEL_MAH = 500      # The absolute minimum charge level where ROS is safe to run.
MIN_VOLTAGE_LEVEL_MV = 6600     # The absolute minimum voltage level where ROS is safe to run.

class Governor():
    def checkPower(self, chargeLevel, voltageLevel):
        """
        This function is responsible for checking to see if:
            * remaining charge is at a critical level (arbitrarily chosen at 500 mAH)
            * if voltage dips to 10% above of what our power converter needs (6.6V)
        If either of these cases occurs, it constitutes an emergency and ROS needs
        to be stopped.
        """
        if chargeLevel < MIN_CHARGE_LEVEL_MAH:
            rospy.logerror("Charge is at critical level! shutting down!")
            proc = subprocess.call(['sudo','/sbin/shutdown','-h', '+1'])
            rospy.signal_shutdown("Charge too low!")
        if voltageLevel < MIN_VOLTAGE_LEVEL_MV:
            rospy.logerror("Voltage is at critical level! shutting down!")
            subprocess.call(['sudo','/sbin/shutdown','-h', '+1'])
            rospy.signal_shutdown("Voltage too low!")

    def checkButtons(self, playButton, advanceButton):
        """
        This function is responsible for sensing and reacting to the play button
        and advance button being hit.
        
        Hitting both of these buttons will lead to a full OS shutdown.
        """
        if playButton and advanceButton:
            rospy.loginfo("OS SHUTDOWN REQUESTED")
            rospy.loginfo("use sudo killall shutdown immediately if this was a mistake")
            proc = subprocess.call(['sudo','/sbin/shutdown','-h', '+1'])
            rospy.signal_shutdown("Shutdown was requested")

    def sensorCallback(self, msg):
        self.checkPower(msg.batteryCharge, msg.voltage)
        self.checkButtons(msg.play, msg.advance)

    def onShutdown(self):
        rospy.loginfo("Shutting Down")

    def start(self):
        rospy.init_node("governor")
        rospy.Subscriber("sensorPacket", SensorPacket, self.sensorCallback)
        rospy.on_shutdown(self.onShutdown)
        rospy.loginfo("Governor Node Online")
        rospy.spin()

if __name__ == '__main__':
    try:
        gov = Governor()
        gov.start()
    except rospy.ROSInterruptException:
        pass

