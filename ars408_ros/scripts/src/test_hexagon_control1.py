#!/usr/bin/env python3

import rospy
import enum

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool

''' DEFINE '''
BUTTON_PRESSED = 1.0
BUTTON_DEPRESSED = 0.0
class CmdSlot(enum.IntFlag):
    steering_value = 0
    headlight_change = 1
    turn_signal_cmd = 2
    brake_value = 3
    accelerator_value = 4
    shift_cmd_park = 5
    shift_cmd_neutral = 6
    shift_cmd_drive = 7
    shift_cmd_reverse = 8
    horn_cmd = 9
    engagement = 10
    disengagement = 11
    wiper_change = 12
    hazards_cmd = 13
    lastslot = 14 
    
# print(CmdSlot.steering_value.value)
class CmdControlRemote:
    def __init__(self):
        rospy.loginfo("CmdControlRemote::Init in")
        rospy.init_node('apitest', anonymous=True)
        # Publisher
        self.user_cmd_pub_ = rospy.Publisher('user_cmd', Float32MultiArray, queue_size=20)
        # Subscriber
        #self.speed_sub_ = rospy.Subscriber("parsed_tx/vehicle_speed_rpt", Bool, VehicleSpeedCb)
        #self.lights_sub_ = rospy.Subscriber("parsed_tx/headlight_rpt", Bool, LightsRptCb)
        self.enable_sub_ = rospy.Subscriber("as_tx/enabled", Bool, self.PacmodEnabledCb)
        #self.user_cmd_ack_ = rospy.Subscriber("user_cmd_ack", Bool, UserCmdAckCb)
        
        self.pacmod_enabled_rpt_ = False
        
        
    def Init(self):
        pass
    
    def LoadPara(self):
        pass
    
    def VehicleSpeedCb(self):
        pass
    
    def PacmodEnabledCb(self):
        pass
    
    def LightsRptCb(self):
        pass
    
    def UserCmdAckCb(self):
        pass
    
    
    
    def KeyRelease(self):
        pass
    
    def KeyBoardState(self):
        pass
    
    def PacmodEnabledCb(self, msg):
        prev_pacmod_enabled_rpt = self.pacmod_enabled_rpt_
        self.pacmod_enabled_rpt_ = msg.data

    def KeyRelease(self, cmd):
        #cmdarray = list(cmd)
        cmd[CmdSlot.shift_cmd_park.value] = BUTTON_DEPRESSED
        cmd[CmdSlot.shift_cmd_neutral.value] = BUTTON_DEPRESSED
        cmd[CmdSlot.shift_cmd_drive.value] = BUTTON_DEPRESSED
        cmd[CmdSlot.shift_cmd_reverse.value] = BUTTON_DEPRESSED
        cmd[CmdSlot.engagement.value] = BUTTON_DEPRESSED
        cmd[CmdSlot.disengagement.value] = BUTTON_DEPRESSED
        cmd[CmdSlot.headlight_change.value] = BUTTON_DEPRESSED

        return Float32MultiArray(data=cmd)
        
    def publishCMD(self):
        cmd = Float32MultiArray()
        cmdarray = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        while not rospy.is_shutdown():
            num = input('Please Input a Command : ')
            if num == '0': # connect car control
                cmdarray[CmdSlot.engagement] = BUTTON_PRESSED
                cmd = Float32MultiArray(data = cmdarray)
                rospy.loginfo(cmd)
                self.user_cmd_pub_.publish(cmd)
                cmd = self.KeyRelease(cmdarray)
                rospy.loginfo(cmd)
                self.user_cmd_pub_.publish(cmd)
                
            if num == '1': # disconnect car control
                cmdarray[CmdSlot.disengagement] = BUTTON_PRESSED
                cmd = Float32MultiArray(data = cmdarray)
                rospy.loginfo(cmd)
                self.user_cmd_pub_.publish(cmd)
                cmd = self.KeyRelease(cmdarray)
                rospy.loginfo(cmd)
                self.user_cmd_pub_.publish(cmd)
            
            if num == '2': # disconnect car control
                cmdarray[CmdSlot.disengagement] = BUTTON_PRESSED
                cmd = Float32MultiArray(data = cmdarray)
                rospy.loginfo(cmd)
                self.user_cmd_pub_.publish(cmd)
                cmd = self.KeyRelease(cmdarray)
                rospy.loginfo(cmd)
                self.user_cmd_pub_.publish(cmd)
            
            if num == '3': # turn_signal_cmd(left)
                if cmdarray[CmdSlot.turn_signal_cmd] == 1.0:
                    cmdarray[CmdSlot.turn_signal_cmd] = 0.0
                else:
                    cmdarray[CmdSlot.turn_signal_cmd] = 1.0

                cmd = Float32MultiArray(data = cmdarray)
                rospy.loginfo(cmd)
                self.user_cmd_pub_.publish(cmd)
                cmd = self.KeyRelease(cmdarray)
                rospy.loginfo(cmd)
                self.user_cmd_pub_.publish(cmd)

            if num == '4': # turn_signal_cmd(right)
                if cmdarray[CmdSlot.turn_signal_cmd] == -1.0:
                    cmdarray[CmdSlot.turn_signal_cmd] = 0.0
                else:
                    cmdarray[CmdSlot.turn_signal_cmd] = -1.0

                cmd = Float32MultiArray(data = cmdarray)
                rospy.loginfo(cmd)
                self.user_cmd_pub_.publish(cmd)
                cmd = self.KeyRelease(cmdarray)
                rospy.loginfo(cmd)
                self.user_cmd_pub_.publish(cmd)
                
            if num == '5': # turn_signal_cmd
                cmdarray[CmdSlot.headlight_change] = BUTTON_PRESSED
                cmd = Float32MultiArray(data = cmdarray)
                rospy.loginfo(cmd)
                self.user_cmd_pub_.publish(cmd)
                cmd = self.KeyRelease(cmdarray)
                rospy.loginfo(cmd)
                self.user_cmd_pub_.publish(cmd)
                
            if num == '6': # steering_value
                if cmdarray[CmdSlot.steering_value] == -0.1:
                    cmdarray[CmdSlot.steering_value] = 0.0
                else:
                    cmdarray[CmdSlot.steering_value] = -0.1

                cmd = Float32MultiArray(data = cmdarray)
                rospy.loginfo(cmd)
                self.user_cmd_pub_.publish(cmd)
                cmd = self.KeyRelease(cmdarray)
                rospy.loginfo(cmd)
                self.user_cmd_pub_.publish(cmd)
            '''
            if num == '7': # steering_value
                if cmdarray[CmdSlot.accelerator_value] == 0.1:
                    cmdarray[CmdSlot.accelerator_value] = 0.0
                else:
                    cmdarray[CmdSlot.accelerator_value] = 0.1

                cmd = Float32MultiArray(data = cmdarray)
                rospy.loginfo(cmd)
                self.user_cmd_pub_.publish(cmd)
                cmd = self.KeyRelease(cmdarray)
                rospy.loginfo(cmd)
                self.user_cmd_pub_.publish(cmd)
            '''
if __name__ == '__main__':
    controllUnit = CmdControlRemote()
    controllUnit.publishCMD()

