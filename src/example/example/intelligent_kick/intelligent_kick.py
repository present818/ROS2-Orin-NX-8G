#!/usr/bin/env python3
# encoding: utf-8

# 智能踢球
import time
import rclpy
import signal
import threading
import sdk.pid as pid
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from arm_kinematics import transform
from arm_kinematics_msgs.srv import SetRobotPose
from rclpy.executors import MultiThreadedExecutor
from interfaces.msg import ColorsInfo, ColorDetect
from servo_controller_msgs.msg import ServosPosition
from rclpy.callback_groups import ReentrantCallbackGroup
from interfaces.srv import SetColorDetectParam, SetString
from controller import step_controller, controller_client
from servo_controller.bus_servo_control import set_servo_position

class IntelligentKickNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.x_dis = 150
        self.y_dis = 500
        self.x_init = transform.link3 + transform.tool_link
        self.center = None
        self.running = True
        self.start = False
        self.kick = False
        self.kick_11 = False
        self.name = name
        self.x_direction = 1
        self.y_direction = 1
        self.pid_x = pid.PID(0.02, 0.0, 0.000001)
        self.pid_y = pid.PID(0.035, 0.0001, 0.000001)
        self.pid_X = pid.PID(0.02, 0.0, 0.000001)
        self.pid_Y = pid.PID(0.035, 0.0001, 0.000001)
        signal.signal(signal.SIGINT, self.shutdown)
        self.step_controller = step_controller.StepController()
        self.controller = controller_client.ControllerClient()


        self.joints_pub = self.create_publisher(ServosPosition, '/servo_controller', 1) # 舵机控制(servo control)
        self.cmd_vel_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)  # 底盘控制(chassis control)

        self.create_subscription(ColorsInfo, '/color_detect/color_info', self.get_color_callback, 1)

        timer_cb_group = ReentrantCallbackGroup()
        self.create_service(Trigger, '~/start', self.start_srv_callback) # 进入玩法(enter the game)
        self.create_service(SetString, '~/set_color', self.set_color_srv_callback, callback_group=timer_cb_group) # 设置颜色(set color)
        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()
        self.client = self.create_client(Trigger, '/arm_kinematics/init_finish')
        self.client.wait_for_service()
        self.set_color_client = self.create_client(SetColorDetectParam, '/color_detect/set_param', callback_group=timer_cb_group)
        self.set_color_client.wait_for_service()

        self.arm_kinematics_client = self.create_client(SetRobotPose, '/arm_kinematics/set_pose_target')
        self.arm_kinematics_client.wait_for_service()

        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def init_process(self):
        self.timer.cancel()

        self.init_action()
        if self.get_parameter('start').value:
            self.start_srv_callback(Trigger.Request(), Trigger.Response())
            request = SetString.Request()
            request.data = 'red'
            self.set_color_srv_callback(request, SetString.Response())

        threading.Thread(target=self.main, daemon=True).start()
        # threading.Thread(target=self.kick_ball, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response
    
    def start_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "start color track")
        self.start = True
        response.success = True
        response.message = "start"
        return response
    
    def shutdown(self, signum, frame):
        self.running = False

    def init_action(self):
        # self.step_controller.set_build_in_pose('DEFAULT_POSE', 1)
        set_servo_position(self.joints_pub, 1, ((24, 700), (23, 500), (22, 150), (21, 100), (20, 665), (19, 500)))
        time.sleep(1)

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def set_color_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "set_color")
        msg = SetColorDetectParam.Request()
        msg_red = ColorDetect()
        msg_red.color_name = request.data
        msg_red.detect_type = 'circle'
        msg.data = [msg_red]
        res = self.send_request(self.set_color_client, msg)
        if res.success:
            self.get_logger().info('\033[1;32m%s\033[0m' % 'start_track_%s'%msg_red.color_name)
        else:
            self.get_logger().info('\033[1;32m%s\033[0m' % 'track_fail')
        response.success = True
        response.message = "set_color"
        return response


    def get_color_callback(self, msg):
        if msg.data != []:
            if msg.data[0].radius > 10:
                self.center = msg.data[0]
            else:
                self.center = None 
        else:
            self.center = None
    def kick_ball(self, center):
        dis = 250 + (center.y-200)
        self.controller.traveling(gait=-2, time=1, steps=0)
        self.kick_11 = True
        self.step_controller.set_leg_position(1, (130, 160, -50), 0.3)
        time.sleep(0.5)
        self.step_controller.set_leg_position(1, (dis , 40, -50), 0.3)
        time.sleep(0.5)
        self.step_controller.set_leg_position(1, (160, 140, -70), 0.3)
        time.sleep(0.5)

    def main(self):
        while self.running:
            twist = Twist()
            if self.start:
                if self.center is not None:
                    t1 = time.time()
                    center = self.center

                    self.pid_y.SetPoint = center.width/2 
                    self.pid_y.update(center.x)
                    self.y_dis += self.pid_y.output
                    
                    if self.y_dis < 350:
                        self.y_dis = 350
                    if self.y_dis > 650:
                        self.y_dis = 650
                    if self.x_dis < 150:
                        self.x_dis = 150
                    if self.x_dis > 250:
                        self.x_dis = 250

                    self.pid_x.SetPoint = center.height/2 
                    self.pid_x.update(center.y)
                    self.x_dis += self.pid_x.output

                    t2 = time.time()
                    t = t2 - t1
                    if t < 0.02:
                        time.sleep(0.02 - t)

                    set_servo_position(self.joints_pub, 0.02, ((19, self.y_dis), (22, int(self.x_dis))))
                    self.pid_Y.SetPoint = 500  # 19号舵机的初始值
                    self.pid_Y.update(self.y_dis)
                    self.pid_X.SetPoint = 200  # 22号舵机的初始值
                    self.pid_X.update(self.x_dis)

                    if self.pid_Y.output < -0.5 :
                        twist.angular.z = 0.1
                    elif self.pid_Y.output > 0.5:
                        twist.angular.z = -0.1
                    elif self.pid_X.output < 1 or center.y < 225:
                        twist.linear.x = 0.03
                    elif self.pid_X.output > 1.2 or center.y > 280:
                        twist.linear.x = -0.02
                    elif 300 < center.x < 330 and 225 < center.y < 280:
                        self.kick = True

                    if self.kick:
                        self.kick_ball(center)
                        self.kick = False

                    else:
                        self.cmd_vel_pub.publish(twist)
                    # self.get_logger().info('\033[1;32m%s\033[0m' % self.pid_Y.output)

                else:
                    self.controller.traveling(gait=-2, time=1, steps=0)
                    # 到达左右边界时，纵向移动一次，x 方向翻转
                    if self.x_dis >= 250:
                        self.x_dis = 250
                        self.x_direction = -1  # 改为向左扫
                        self.y_dis += 10 * self.y_direction  # 根据方向向上或向下扫一次

                    elif self.x_dis <= 150:
                        self.x_dis = 150
                        self.x_direction = 1   # 改为向右扫
                        self.y_dis += 10 * self.y_direction

                    # 控制纵向上下边界范围（上下来回）
                    if self.y_dis >= 650:
                        self.y_dis = 650
                        self.y_direction = -1
                        self.x_dis += 20 * self.x_direction
                    elif self.y_dis <= 350:
                        self.y_dis = 350
                        self.y_direction = 1
                        self.x_dis += 20 * self.x_direction

                    # 设置舵机位置
                    set_servo_position(self.joints_pub, 0.2, ((19, int(self.y_dis)), (22, int(self.x_dis))))
                    time.sleep(0.2)

        self.init_action()
        rclpy.shutdown()

def main():
    node = IntelligentKickNode('intelligent_kick')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
 
if __name__ == "__main__":
    main()
