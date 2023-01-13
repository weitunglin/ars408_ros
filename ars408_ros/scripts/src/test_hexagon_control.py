#!/usr/bin/env python3

from pynput import keyboard
import rospy
import enum

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool


import threading
mutex = threading.Lock()

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
        self.cmd_list = [0.0]*15


        keyboard_listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        keyboard_listener.start()

    def on_press(self, key):
        global mutex
        try:
            print('alphanumeric key {0} pressed'.format(key))
            if key == keyboard.Key.esc: # disconnect Hexagon
                print('--------------disconnect Hexagon------------------')
                mutex.acquire()
                cmdarray = [0.0] * 15
                cmdarray[CmdSlot.disengagement.value] = 1.0
                cmd = Float32MultiArray(data = cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                cmd = self.KeyRelease(cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                self.cmd_list = cmd.data
                mutex.release()
                

            if key.char == '0': # connect Hexagon
                print('----------------connect Hexagon--------------------')
                mutex.acquire()
                cmdarray = self.cmd_list
                cmdarray[CmdSlot.engagement.value] = 1.0
                cmd = Float32MultiArray(data = cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                cmd = self.KeyRelease(cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                self.cmd_list = cmd.data
                mutex.release()
            
            if key.char == 'w': # accelerator_value ++ (speed up)
                print('----------------speed up--------------------')
                mutex.acquire()
                cmdarray = self.cmd_list
                acv = cmdarray[CmdSlot.accelerator_value.value] + 0.01
                veh_accelerator_value = 0.3
                cmdarray[CmdSlot.accelerator_value.value] = veh_accelerator_value if acv > veh_accelerator_value else acv
                cmdarray[CmdSlot.brake_value.value] = 0.0
                cmd = Float32MultiArray(data = cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                cmd = self.KeyRelease(cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                self.cmd_list = cmd.data
                mutex.release()


            if key.char == 'a': # steering_value ++ (steering left)
                print('----------------steering left--------------------')
                mutex.acquire()
                cmdarray = self.cmd_list
                stev = cmdarray[CmdSlot.steering_value.value] + 0.01
                veh_max_steering_val = 0.2
                cmdarray[CmdSlot.steering_value.value] = veh_max_steering_val if stev > veh_max_steering_val else stev
                cmd = Float32MultiArray(data = cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                cmd = self.KeyRelease(cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                self.cmd_list = cmd.data
                mutex.release()

            if key.char == 's': # accelerator_value -- (speed down)
                print('----------------speed down--------------------')
                mutex.acquire()
                cmdarray = self.cmd_list
                acv = cmdarray[CmdSlot.accelerator_value.value] - 0.02
                cmdarray[CmdSlot.accelerator_value.value] = 0 if acv < 0.0 else acv
                cmdarray[CmdSlot.brake_value.value] = 0.0
                cmd = Float32MultiArray(data = cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                cmd = self.KeyRelease(cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                self.cmd_list = cmd.data
                mutex.release()

            if key.char == 'd': # steering_value -- (turn right)
                print('----------------steering right--------------------')
                mutex.acquire()
                cmdarray = self.cmd_list
                stev = cmdarray[CmdSlot.steering_value.value] - 0.01
                veh_max_steering_val = 0.2
                cmdarray[CmdSlot.steering_value.value] = -(veh_max_steering_val) if stev < -(veh_max_steering_val) else stev
                cmd = Float32MultiArray(data = cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                cmd = self.KeyRelease(cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                self.cmd_list = cmd.data
                mutex.release()
                

            if key.char == 'f': # steering_value = 0.0
                print('------------steering back----------------')
                mutex.acquire()
                cmdarray = self.cmd_list
                cmdarray[CmdSlot.steering_value.value] = 0.0
                cmd = Float32MultiArray(data = cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                cmd = self.KeyRelease(cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                self.cmd_list = cmd.data
                mutex.release()
            
            if key.char == 'p': # shift_cmd_park
                print('------------PARK----------------')
                print('WARNING : shift_cmd_park is not available')
                '''
                mutex.acquire()
                cmdarray = self.cmd_list
                cmdarray = [0.0] * 15
                cmdarray[CmdSlot.shift_cmd_park.value] = BUTTON_PRESSED
                cmd = Float32MultiArray(data = cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                cmd = self.KeyRelease(cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                self.cmd_list = cmd.data
                mutex.release()
                '''
                pass

            if key.char == 'n': # shift_cmd_neutral
                print('------------NONE----------------')
                print('WARNING : shift_cmd_neutral is not available')
                '''
                mutex.acquire()
                cmdarray = self.cmd_list
                cmdarray = [0.0] * 15
                cmdarray[CmdSlot.shift_cmd_neutral.value] = BUTTON_PRESSED
                cmd = Float32MultiArray(data = cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                cmd = self.KeyRelease(cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                self.cmd_list = cmd.data
                mutex.release()
                '''
                pass

            if key.char == 'r': # shift_cmd_reverse
                print('------------REVERSE----------------')
                print('WARNING : shift_cmd_reverse is not available')
                '''
                mutex.acquire()
                cmdarray = self.cmd_list
                cmdarray = [0.0] * 15
                cmdarray[CmdSlot.shift_cmd_reverse.value] = BUTTON_PRESSED
                cmd = Float32MultiArray(data = cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                cmd = self.KeyRelease(cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                self.cmd_list = cmd.data
                mutex.release()
                '''
                pass

            if key.char == 'g': # shift_cmd_drive = 0.0
                print('------------DRIVE----------------')
                print('WARNING : shift_cmd_drive is not available')
                '''
                mutex.acquire()
                cmdarray = self.cmd_list
                cmdarray = [0.0] * 15
                cmdarray[CmdSlot.shift_cmd_drive.value] = BUTTON_PRESSED
                cmd = Float32MultiArray(data = cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                cmd = self.KeyRelease(cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                self.cmd_list = cmd.data
                mutex.release()
                '''
                pass

            if key.char == '1': # turn_signal_cmd = 1.0 (left light)
                print('------------left light----------------')
                mutex.acquire()
                cmdarray = self.cmd_list
                cmdarray[CmdSlot.turn_signal_cmd.value] = 1.0 if cmdarray[CmdSlot.turn_signal_cmd.value] != 1.0 else 0.0
                cmd = Float32MultiArray(data = cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                cmd = self.KeyRelease(cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                self.cmd_list = cmd.data
                mutex.release()
                

            if key.char == '4': # turn_signal_cmd = -1.0 (right light)
                print('------------right light----------------')
                mutex.acquire()
                cmdarray = self.cmd_list
                cmdarray[CmdSlot.turn_signal_cmd.value] = -1.0 if cmdarray[CmdSlot.turn_signal_cmd.value] != -1.0 else 0.0
                cmd = Float32MultiArray(data = cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                cmd = self.KeyRelease(cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                self.cmd_list = cmd.data
                mutex.release()

            if key.char == 'l': # wiper_change (head light)
                print('------------head light change----------------')
                mutex.acquire()
                cmdarray = self.cmd_list
                cmdarray[CmdSlot.headlight_change.value] = 1.0
                cmd = Float32MultiArray(data = cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                cmd = self.KeyRelease(cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                self.cmd_list = cmd.data
                mutex.release()

            if key.char == '8': # horn_cmd (horn)
                print('------------horn----------------')
                mutex.acquire()
                cmdarray = self.cmd_list
                cmdarray[CmdSlot.horn_cmd.value] = BUTTON_PRESSED if cmdarray[CmdSlot.horn_cmd.value] == BUTTON_DEPRESSED else BUTTON_DEPRESSED
                cmd = Float32MultiArray(data = cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                cmd = self.KeyRelease(cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                self.cmd_list = cmd.data
                mutex.release()
                
        except AttributeError:
            print('special key {0} pressed'.format(key))
            if key == keyboard.Key.space: # brake
                print('----------------brake--------------------')
                mutex.acquire()
                cmdarray = self.cmd_list
                bkv = cmdarray[CmdSlot.brake_value.value] + 0.02
                veh_max_break_val = 0.4
                cmdarray[CmdSlot.brake_value.value] = veh_max_break_val if bkv > veh_max_break_val else bkv
                cmdarray[CmdSlot.accelerator_value.value] = 0.0
                cmd = Float32MultiArray(data = cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                cmd = self.KeyRelease(cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                self.cmd_list = cmd.data
                mutex.release()
                
                

    def on_release(self, key):
        print('{0} released'.format(key))
        pass


        
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
        cmdarray = self.cmd_list
        while not rospy.is_shutdown():
            num = input('Please Input a Command : ')
            if num == '0': # connect car control
                cmdarray[CmdSlot.engagement.value] = BUTTON_PRESSED
                cmd = Float32MultiArray(data = cmdarray)
                rospy.loginfo(cmd)
                self.user_cmd_pub_.publish(cmd)
                cmd = self.KeyRelease(cmdarray)
                rospy.loginfo(cmd)
                self.user_cmd_pub_.publish(cmd)
                
            if num == '1': # disconnect car control
                cmdarray[CmdSlot.disengagement.value] = BUTTON_PRESSED
                cmd = Float32MultiArray(data = cmdarray)
                rospy.loginfo(cmd)
                self.user_cmd_pub_.publish(cmd)
                cmd = self.KeyRelease(cmdarray)
                rospy.loginfo(cmd)
                self.user_cmd_pub_.publish(cmd)
            
            if num == '2': # turn_signal_cmd (left)
                if cmdarray[CmdSlot.turn_signal_cmd.value] == 1.0:
                    cmdarray[CmdSlot.turn_signal_cmd.value] = 0.0
                else:
                    cmdarray[CmdSlot.turn_signal_cmd.value] = 1.0

                cmd = Float32MultiArray(data = cmdarray)
                rospy.loginfo(cmd)
                self.user_cmd_pub_.publish(cmd)
                cmd = self.KeyRelease(cmdarray)
                rospy.loginfo(cmd)
                self.user_cmd_pub_.publish(cmd)

            if num == '3': # turn_signal_cmd (right)
                if cmdarray[CmdSlot.turn_signal_cmd.value] == -1.0:
                    cmdarray[CmdSlot.turn_signal_cmd.value] = 0.0
                else:
                    cmdarray[CmdSlot.turn_signal_cmd.value] = -1.0

                cmd = Float32MultiArray(data = cmdarray)
                rospy.loginfo(cmd)
                self.user_cmd_pub_.publish(cmd)
                cmd = self.KeyRelease(cmdarray)
                rospy.loginfo(cmd)
                self.user_cmd_pub_.publish(cmd)

            if num == '4': # turn_signal_cmd (right)
                if cmdarray[CmdSlot.brake_value.value] == -1.0:
                    cmdarray[CmdSlot.brake_value.value] = 0.0
                else:
                    cmdarray[CmdSlot.brake_value.value] = -1.0

                cmd = Float32MultiArray(data = cmdarray)
                rospy.loginfo(cmd)
                self.user_cmd_pub_.publish(cmd)
                cmd = self.KeyRelease(cmdarray)
                rospy.loginfo(cmd)
                self.user_cmd_pub_.publish(cmd)

    def turn_left():
        global mutex

    def turn_right():
        global mutex

    def turn_zero():
        global mutex

    def speedup():
        global mutex

    def speeddown():
        global mutex

    def speedzero():
        global mutex

    def brake():
        global mutex





if __name__ == '__main__':
    controllUnit = CmdControlRemote()
    controllUnit.publishCMD()

