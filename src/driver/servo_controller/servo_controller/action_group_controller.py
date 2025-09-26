#!/usr/bin/env python3
# encoding: utf-8
import os
import time
import threading
import sqlite3 as sql
from servo_controller_msgs.msg import ServosPosition, ServoPosition

class ActionGroupController:
    running_action = False
    stop_running = False
    def __init__(self, pub, action_path):
        self.servo_controller_pub = pub
        self.action_path = action_path

    def start_action_thread(self, actNum, lock_servos=''):
        """启动一个新线程来运行动作组，避免阻塞主程序"""
        if self.running_action:
            return

        # 创建并启动线程
        action_thread = threading.Thread(target=self.run_action, args=(actNum, lock_servos))
        action_thread.daemon = True # 设置为守护线程，主程序退出时它也会退出
        action_thread.start()

    def stop_action_group(self):
        self.stop_running = True

    def run_action(self, actNum,lock_servos=''):   
        if self.running_action:
            return

        if actNum is None:
            return
        
        if actNum == 'body_wave':
            self.body_wave()
        else:
            actNum = os.path.join(self.action_path, actNum + ".d6a")

            self.stop_running= False
            if os.path.exists(actNum) is True:
                if self.running_action is False:
                    self.running_action = True
                    ag = sql.connect(actNum)
                    cu = ag.cursor()
                    cu.execute("select * from ActionGroup")
                    
                    while True:
                        act = cu.fetchone()
                        if self.stop_running is True:
                            self.stop_running= False                   
                            break

                        if act is not None:
                            data = []
                            msg = ServosPosition()
                            msg.position_unit = 'pulse'
                            msg.duration = float(act[1])/1000.0
                            for i in range(0, len(act) - 2, 1):
                                servo = ServoPosition()
                                servo.id = i + 1
                                if str(servo.id)  in lock_servos:
                                    servo.position = float(lock_servos[str(servo.id)])
                                else:
                                    servo.position = float(act[2 + i])
                                data.append(servo) 
                            msg.position = data
                            self.servo_controller_pub.publish(msg)
                            time.sleep(float(act[1])/1000.0)                       

                        else:   # 运行完才退出
                            break
                    self.running_action = False
                    
                    cu.close()
                    ag.close()
            else:
                self.running_action = False
                print('Unable to find action group file:',self.action_path)

