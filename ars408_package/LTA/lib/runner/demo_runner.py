
import torch
import numpy as np
from tqdm import tqdm
import cv2
from .registry import build_evaluator
from .optimizer import build_optimizer
from .scheduler import build_scheduler
from .recorder import build_recorder
from .net_utils import  load_network
#user cdoe
import torchvision
import lib.utils_resa.transforms as tf

#Car Control Code
import rospy
from pacmod_msgs.msg import VehicleSpeedRpt
##

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool

import enum

from pynput import keyboard
import threading
mutex = threading.Lock()

BUTTON_PRESSED = 1.0
BUTTON_DEPRESSED = 0.0
"""
用來跑Demo資料的程式：

使用方法：
demo_runner = DemoString(path,Net,weight,device,cfg_resa,source_type)
demo_runner.run()
"""



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
"""
class CmdControlRemote:
    def __init__(self):
        rospy.loginfo("CmdControlRemote::Init in")
        # rospy.init_node('apitest', anonymous=True)
        # Publisher
        ''' DEFINE '''
        self.BUTTON_PRESSED = 1.0
        self.BUTTON_DEPRESSED = 0.0
        self.user_cmd_pub_ = rospy.Publisher('user_cmd', Float32MultiArray, queue_size=20)
        # Subscriber
        #self.speed_sub_ = rospy.Subscriber("parsed_tx/vehicle_speed_rpt", Bool, VehicleSpeedCb)
        #self.lights_sub_ = rospy.Subscriber("parsed_tx/headlight_rpt", Bool, LightsRptCb)
        self.enable_sub_ = rospy.Subscriber("as_tx/enabled", Bool, self.PacmodEnabledCb)
        #self.user_cmd_ack_ = rospy.Subscriber("user_cmd_ack", Bool, UserCmdAckCb)
        
        self.pacmod_enabled_rpt_ = False
        #self.enableHexagon()
        self.cmd_list = [0] * 10
        
        
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
        cmd[CmdSlot.shift_cmd_park.value] = self.BUTTON_DEPRESSED
        cmd[CmdSlot.shift_cmd_neutral.value] = self.BUTTON_DEPRESSED
        cmd[CmdSlot.shift_cmd_drive.value] = self.BUTTON_DEPRESSED
        cmd[CmdSlot.shift_cmd_reverse.value] = self.BUTTON_DEPRESSED
        cmd[CmdSlot.engagement.value] = self.BUTTON_DEPRESSED
        cmd[CmdSlot.disengagement.value] = self.BUTTON_DEPRESSED
        cmd[CmdSlot.headlight_change.value] = self.BUTTON_DEPRESSED

        return Float32MultiArray(data=cmd)
        
    def publishCMD(self):
        cmd = Float32MultiArray()
        cmdarray = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        while not rospy.is_shutdown():
            num = input('Please Input a Command : ')
            if num == '0': # connect car control
                cmdarray[CmdSlot.engagement] = self.BUTTON_PRESSED
                cmd = Float32MultiArray(data = cmdarray)
                rospy.loginfo(cmd)
                self.user_cmd_pub_.publish(cmd)
                cmd = self.KeyRelease(cmdarray)
                rospy.loginfo(cmd)
                self.user_cmd_pub_.publish(cmd)
                
            if num == '1': # disconnect car control
                cmdarray[CmdSlot.disengagement] = self.BUTTON_PRESSED
                cmd = Float32MultiArray(data = cmdarray)
                rospy.loginfo(cmd)
                self.user_cmd_pub_.publish(cmd)
                cmd = self.KeyRelease(cmdarray)
                rospy.loginfo(cmd)
                self.user_cmd_pub_.publish(cmd)
            
            if num == '2': # disconnect car control
                cmdarray[CmdSlot.disengagement] = self.BUTTON_PRESSED
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
                cmdarray[CmdSlot.headlight_change] = self.BUTTON_PRESSED
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


    def steering_left(self):
        cmd = Float32MultiArray()
        cmdarray = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        
        cmdarray[CmdSlot.steering_value] = 0.1

        cmd = Float32MultiArray(data = cmdarray)
        rospy.loginfo(cmd)
        self.user_cmd_pub_.publish(cmd)
        cmd = self.KeyRelease(cmdarray)
        rospy.loginfo(cmd)
        self.user_cmd_pub_.publish(cmd)
    
    def steering_right(self):
        cmd = Float32MultiArray()
        cmdarray = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        
        cmdarray[CmdSlot.steering_value] = -0.1

        cmd = Float32MultiArray(data = cmdarray)
        rospy.loginfo(cmd)
        self.user_cmd_pub_.publish(cmd)
        cmd = self.KeyRelease(cmdarray)
        rospy.loginfo(cmd)
        self.user_cmd_pub_.publish(cmd)

    def steering_back(self):
        cmd = Float32MultiArray()
        cmdarray = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        
        

        cmd = Float32MultiArray(data = cmdarray)
        rospy.loginfo(cmd)
        self.user_cmd_pub_.publish(cmd)
        cmd = self.KeyRelease(cmdarray)
        rospy.loginfo(cmd)
        self.user_cmd_pub_.publish(cmd)
 


    def turn_left(self):
        cmd = Float32MultiArray()
        cmdarray = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        cmdarray[CmdSlot.turn_signal_cmd] = 1.0

        cmd = Float32MultiArray(data = cmdarray)
        rospy.loginfo(cmd)
        self.user_cmd_pub_.publish(cmd)
        cmd = self.KeyRelease(cmdarray)
        rospy.loginfo(cmd)
        self.user_cmd_pub_.publish(cmd)

    def turn_right(self):
        cmd = Float32MultiArray()
        cmdarray = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmdarray[CmdSlot.turn_signal_cmd] = -1.0
        self.cmd_list = cmdarray

        cmd = Float32MultiArray(data = cmdarray)
        rospy.loginfo(cmd)
        self.user_cmd_pub_.publish(cmd)
        cmd = self.KeyRelease(cmdarray)
        rospy.loginfo(cmd)
        self.user_cmd_pub_.publish(cmd)
        
    def disable_turn_signal(self):
        cmd = Float32MultiArray()
        cmdarray = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        cmdarray[CmdSlot.turn_signal_cmd] = 0.0
        self.cmd_list = cmdarray

        cmd = Float32MultiArray(data = cmdarray)
        rospy.loginfo(cmd)
        self.user_cmd_pub_.publish(cmd)
        cmd = self.KeyRelease(cmdarray)
        rospy.loginfo(cmd)
        self.user_cmd_pub_.publish(cmd)

    def enableHexagon(self):
        cmd = Float32MultiArray()
        cmdarray = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmdarray[CmdSlot.engagement.value] = self.BUTTON_PRESSED
        cmd = Float32MultiArray(data = cmdarray)
        rospy.loginfo(cmd)
        self.user_cmd_pub_.publish(cmd)
        cmd = self.KeyRelease(cmdarray)
        rospy.loginfo(cmd)
        self.user_cmd_pub_.publish(cmd)

    def disableHexagon(self):
        cmd = Float32MultiArray()
        cmdarray = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmdarray[CmdSlot.disengagement] = self.BUTTON_PRESSED
        cmd = Float32MultiArray(data = cmdarray)
        rospy.loginfo(cmd)
        self.user_cmd_pub_.publish(cmd)
        cmd = self.KeyRelease(cmdarray)
        rospy.loginfo(cmd)
        self.user_cmd_pub_.publish(cmd)

"""
class CmdControlRemote:
    def __init__(self):
        rospy.loginfo("CmdControlRemote::Init in")
        # rospy.init_node('apitest', anonymous=True)
        # Publisher
        self.user_cmd_pub_ = rospy.Publisher('user_cmd', Float32MultiArray, queue_size=20)
        # Subscriber
        #self.speed_sub_ = rospy.Subscriber("parsed_tx/vehicle_speed_rpt", Bool, VehicleSpeedCb)
        #self.lights_sub_ = rospy.Subscriber("parsed_tx/headlight_rpt", Bool, LightsRptCb)
        self.enable_sub_ = rospy.Subscriber("as_tx/enabled", Bool, self.PacmodEnabledCb)
        #self.user_cmd_ack_ = rospy.Subscriber("user_cmd_ack", Bool, UserCmdAckCb)
        
        self.pacmod_enabled_rpt_ = False
        #self.enableHexagon()
        self.cmd_list = [0] * 15

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
        cmdarray = self.cmd_list
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


    def steering_left(self):
        mutex.acquire()
        cmd = Float32MultiArray()
        #cmdarray = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmdarray = self.cmd_list
        
        cmdarray[CmdSlot.steering_value] = 0.075

        cmd = Float32MultiArray(data = cmdarray)
        rospy.loginfo(cmd)
        self.user_cmd_pub_.publish(cmd)
        cmd = self.KeyRelease(cmdarray)
        rospy.loginfo(cmd)
        self.user_cmd_pub_.publish(cmd)
        mutex.release()
    
    def steering_right(self):
        mutex.acquire()
        cmd = Float32MultiArray()
        #cmdarray = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmdarray = self.cmd_list
        
        cmdarray[CmdSlot.steering_value] = -0.075

        cmd = Float32MultiArray(data = cmdarray)
        rospy.loginfo(cmd)
        self.user_cmd_pub_.publish(cmd)
        cmd = self.KeyRelease(cmdarray)
        rospy.loginfo(cmd)
        self.user_cmd_pub_.publish(cmd)
        mutex.release()


    def steering_back(self):
        mutex.acquire()
        cmd = Float32MultiArray()
        cmdarray = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        #cmdarray = self.cmd_list
        cmdarray[CmdSlot.steering_value] = 0
        
        

        cmd = Float32MultiArray(data = cmdarray)
        rospy.loginfo(cmd)
        self.user_cmd_pub_.publish(cmd)
        cmd = self.KeyRelease(cmdarray)
        rospy.loginfo(cmd)
        self.user_cmd_pub_.publish(cmd)
        mutex.release()

    def turn_right(self):
        mutex.acquire()
        cmd = Float32MultiArray()
        cmdarray = self.cmd_list
        cmdarray[CmdSlot.turn_signal_cmd] = -1.0
        cmd = Float32MultiArray(data = cmdarray)
        rospy.loginfo(cmd)
        self.user_cmd_pub_.publish(cmd)
        cmd = self.KeyRelease(cmdarray)
        rospy.loginfo(cmd)
        self.user_cmd_pub_.publish(cmd)
        mutex.release()

    def turn_left(self):
        mutex.acquire()
        cmd = Float32MultiArray()
        cmdarray = self.cmd_list
        cmdarray[CmdSlot.turn_signal_cmd] = 1.0
        self.cmd_list = cmdarray
        cmd = Float32MultiArray(data = cmdarray)
        rospy.loginfo(cmd)
        self.user_cmd_pub_.publish(cmd)
        cmd = self.KeyRelease(cmdarray)
        rospy.loginfo(cmd)
        self.user_cmd_pub_.publish(cmd)
        mutex.release()
        
    def disable_turn_signal(self):
        cmd = Float32MultiArray()
        cmdarray = self.cmd_list

        cmdarray[CmdSlot.turn_signal_cmd] = 0.0
        self.cmd_list = cmdarray

        cmd = Float32MultiArray(data = cmdarray)
        rospy.loginfo(cmd)
        self.user_cmd_pub_.publish(cmd)
        cmd = self.KeyRelease(cmdarray)
        rospy.loginfo(cmd)
        self.user_cmd_pub_.publish(cmd)

    def speed_up(self, speed ):
        global mutex
        mutex.acquire()
        cmdarray = self.cmd_list
        acv = cmdarray[CmdSlot.accelerator_value.value] + 0.00005 * speed
        veh_accelerator_value = 0.2
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
        return acv

    def speed_down(self):
        global mutex

    def speed_zero(self):
        global mutex
        mutex.acquire()
        cmdarray = self.cmd_list
        cmdarray[CmdSlot.accelerator_value.value] = 0.0
        cmd = Float32MultiArray(data = cmdarray)
        rospy.loginfo(cmd.data)
        self.user_cmd_pub_.publish(cmd)
        cmd = self.KeyRelease(cmdarray)
        rospy.loginfo(cmd.data)
        self.user_cmd_pub_.publish(cmd)
        self.cmd_list = cmd.data
        mutex.release()

    def brake(self, speed):
        print('----------------brake--------------------')
        global mutex
        mutex.acquire()
        cmdarray = self.cmd_list
        
        bkv = cmdarray[CmdSlot.brake_value.value] + 0.002 * speed + 0.01
        veh_max_break_val = 0.2
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
        return bkv

    def brake_zero(self):
        print('----------------brake--------------------')
        global mutex
        mutex.acquire()
        cmdarray = self.cmd_list
        cmdarray[CmdSlot.brake_value.value] = 0.0
        cmd = Float32MultiArray(data = cmdarray)
        rospy.loginfo(cmd.data)
        self.user_cmd_pub_.publish(cmd)
        cmd = self.KeyRelease(cmdarray)
        rospy.loginfo(cmd.data)
        self.user_cmd_pub_.publish(cmd)
        self.cmd_list = cmd.data
        mutex.release()

    def enableHexagon(self):
        cmd = Float32MultiArray()
        cmdarray = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmdarray[CmdSlot.engagement.value] = BUTTON_PRESSED
        cmd = Float32MultiArray(data = cmdarray)
        rospy.loginfo(cmd)
        self.user_cmd_pub_.publish(cmd)
        cmd = self.KeyRelease(cmdarray)
        rospy.loginfo(cmd)
        self.user_cmd_pub_.publish(cmd)

    def disableHexagon(self):
        cmd = Float32MultiArray()
        cmdarray = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmdarray[CmdSlot.disengagement] = BUTTON_PRESSED
        cmd = Float32MultiArray(data = cmdarray)
        rospy.loginfo(cmd)
        self.user_cmd_pub_.publish(cmd)
        cmd = self.KeyRelease(cmdarray)
        rospy.loginfo(cmd)
        self.user_cmd_pub_.publish(cmd)


class DemoString:
    def __init__(self,path,Net,weight,device,cfg_resa,source_type, img = None):
        """
        parameter:
        ---------------------
        path:demo資料的資料夾路徑
        Net:使用的網路Model
        weight:使用的.pth檔案
        device:使用的gpu或cpu
        cfg_resa:config檔
        source_type:"image" 或是 "video"
        """
        ## Enable/Disable CarControl
        self.controllUnit = CmdControlRemote()
        
        # self.controllUnit.publishCMD()
        self.cv2_img = img
        self.source_type = source_type
        self.cfg = cfg_resa
        self.device = device
        self.recorder = build_recorder(self.cfg)
        self.path = path.replace("\\","/") if path else None
        self.transformer = torchvision.transforms.Compose([
            tf.SampleResize((640, 384)),
            tf.GroupNormalize(mean=([103.939, 116.779, 123.68], (0, )), std=(
                [1., 1., 1.], (1, ))),
            ])
        self.net = Net
        self.net = torch.nn.parallel.DataParallel(
                self.net, device_ids = range(self.cfg.gpus)).cuda()
        #self.recorder.logger.info('Network: \n' + str(self.net))
        self.resume()
        self.optimizer = build_optimizer(self.cfg, self.net)
        self.scheduler = build_scheduler(self.cfg, self.optimizer)
        self.evaluator = build_evaluator(self.cfg)
        self.weight = weight
        self.device = device
        self.net.load_state_dict(self.weight['net'],strict = False)
        self.net = self.net.to(self.device)

        #Car control Code
        self.current_speed = 0
        self.sub = rospy.Subscriber("/parsed_tx/vehicle_speed_rpt", VehicleSpeedRpt, self.speed_callback, queue_size=20)
        
        #
        
    def resume(self):
        if not self.cfg.load_from and not self.cfg.finetune_from:
            return
        load_network(self.net, self.cfg.load_from,
                finetune_from=self.cfg.finetune_from, logger=self.recorder.logger)

    def run(self):
        # get frame:

        directionSignal = '0'
        TouchSignal = '0'

        if self.source_type.lower() == "video":
            vidcap = cv2.VideoCapture(self.path)
            success,image = vidcap.read()
            h,w,_ = image.shape
            size = (w,h)
            count = 0
            result_img = []
            (major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')
            #得到影片的FPS
            if int(major_ver)  < 3 :
                fps = vidcap.get(cv2.cv.CV_CAP_PROP_FPS)
                print("Frames per second using video.get(cv2.cv.CV_CAP_PROP_FPS):{0}".format(fps))
            else :
                fps = vidcap.get(cv2.CAP_PROP_FPS)
                print("Frames per second using video.get(cv2.CAP_PROP_FPS) : {0}".format(fps))
            
            while success: # 將影片一張一張地抽禎
                print("frame ",str(count+1))
                #cv2.imwrite("frame%d.jpg" % count, image)     # save frame as JPEG file      
                img = self.transform(image)# 將抽禎出來的照片前處理
                
                img = img.to(self.device)
                
                with torch.no_grad():
                    output = self.net(img)[0]# 將抽禎出來的照片丟入網路
                    img = output['seg'].cpu().numpy()
                    
                    try:

                        if (self.current_speed > 60):

                            # 將輸出結果作後處理，並且印出車道線、箭頭等等
                            result = self.evaluator.demo(self.path, output, img ,ori_image = image)
                            #這裡的evaluator 其實就是 lib/runner/evaluator/tusimple/tusimple.py這個文件裡面的class TuSimple_Demo

                            result_img.append(result)
                    except :
                        pass
                    
                    
                        
                success,image = vidcap.read()
                count += 1

                

            self.out_vid(result_img,size,fps)
        elif self.source_type.lower() == "image":
            import time,os
            a = time.localtime()
            re = time.strftime("-%Y-%m-%d", a)
            print("load image from {}...".format(self.path))
            data_list = os.listdir(self.path)
            save_dir = "./demo/"+ self.path.split('/')[-1] + re

            if not os.path.exists(save_dir ):
                os.makedirs(save_dir )       
            t = tqdm(total = len(data_list))

            for path in data_list: #將資料夾的照片一張一張拿去預測
                p = self.path +"/{}".format(path)
                image = cv2.imread(p)
                img = self.transform(image)
                img = img.to(self.device)

                with torch.no_grad():
                    output = self.net(img)[0]
                    img = output['seg'].cpu().numpy()
                    result = self.evaluator.demo(self.path, output, img ,ori_image = image)
                    #這裡的evaluator 其實就是 lib/runner/evaluator/tusimple/tusimple.py這個文件裡面的class TuSimple_Demo
                    #cv2.imshow('self.road_image',result)
                    
                    
                self.out_img(img = result,img_name = path,save_dir =save_dir)
                t.update(1)

        elif self.source_type.lower() == "ros":
            img = self.transform(self.cv2_img)
            #print("1")
            
            
            #cv2.imshow("lta", self.cv2_img)
            #if cv2.waitKey(1) == ord('q'):  # q to quit
            #    raise StopIteration
            
            img = img.to(self.device)
            with torch.no_grad():
                output = self.net(img)[0]
                img = output['seg'].cpu().numpy()

                # if (self.current_speed):
                    
                #     result = self.evaluator.demo(self.path , output, img ,ori_image = self.cv2_img)
                    
                
                # else:
                    
                #     result = img

                
                    
                result, directionSignal, TouchSignal= self.evaluator.demo(self.path , output, img ,ori_image = self.cv2_img)

                
                print("SIGNAL:"+ directionSignal)

                System = 1  # 0 is TOUCHLine //  1 is KeepCenter
                if (System == 1):
                #### "KEEPCENTER" Control Signal    ####
                    if directionSignal == "KeepLeft":
                        self.controllUnit.turn_left()
                        print("TURNLEFTTTTTTTTTTTTTTTTT")
                    elif directionSignal == "KeepRight":
                        self.controllUnit.turn_right()
                        print("TURNRIGHTTTTTTTTTTTTTTTTTTTT")
                    else:
                        self.controllUnit.disable_turn_signal()
                        print("NOTURNNNNNNNNNNNNNNNNNNN")

                
                
                    ####  "KEEPCENTER" Control Signal   ####
                    if directionSignal == "KeepLeft":
                        self.controllUnit.steering_left()
                    elif directionSignal == "KeepRight":
                        self.controllUnit.steering_right()
                    else:
                        self.controllUnit.steering_back()
                    
                if (System == 0):
                    print("TOUCHSIGNAL: " + TouchSignal[0])

                    #### "TOUCHLINE" Control Signal ####
                    if TouchSignal[0] == "R":
                        self.controllUnit.steering_left()
                        self.controllUnit.turn_left()
                    elif TouchSignal[0] == "L":
                        self.controllUnit.steering_right()
                        self.controllUnit.turn_right()
                    else:
                        self.controllUnit.disable_turn_signal()
                        self.controllUnit.steering_back()
                    
                
                
                    
                #result = img
                    
            
     
            realSpeedText = 'Current Speed : ' + str(int(self.current_speed)) + ' km/h'
            cv2.putText(self.cv2_img, realSpeedText, (0, 670), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
            cv2.imshow("lta", self.cv2_img)
            if cv2.waitKey(1) == ord('q'):  # q to quit
                raise StopIteration


            return result
               
                        
            

        else:
            raise Exception("invalid source_type")
    def out_img(self,img,img_name,save_dir):#把圖片儲存起來
        cv2.imwrite(save_dir+"/"+img_name, img)
    def out_vid(self,img_list,size,fps):#把影片儲存起來
        import time,os
        a = time.localtime()
        re = time.strftime("-%Y-%m-%d", a)
    
        video_name = self.path.split("/")[-1]
        video_name = video_name+re
        model_dir = "./demo/{}".format(video_name)
        
        os.makedirs(model_dir)        
        file_name = self.path.split('/')[-1][:-4]
        out = cv2.VideoWriter("{}/{}.avi".format(model_dir,file_name),cv2.VideoWriter_fourcc(*'DIVX'),fps,size)
        for i in range(len(img_list)):
            out.write(img_list[i])
        out.release()
    def transform(self,image):
        image = image[160:, :, :]
        image, = self.transformer((image,))
        image = torch.from_numpy(image).permute(2, 0, 1).contiguous().float()
        image = image.unsqueeze(0)
        return image       

    ##Car Control Code
    def speed_callback(self, msg: VehicleSpeedRpt):
        if not msg.vehicle_speed_valid:
            rospy.loginfo(msg.vehicle_speed_valid)
            #print(type(msg.vehicle_speed_valid))
            
        else:
            #rospy.loginfo(msg.vehicle_speed * 3.6)
            self.current_speed = msg.vehicle_speed * 3.6
            print( self.current_speed )
    ##