#!/usr/bin/env python3
# encoding: utf-8

import math
import time
import rclpy
import threading
import numpy as np
from rclpy.node import Node
from interfaces.msg import RunActionSet
from kinematics import kinematics_calculate
from rclpy.executors import MultiThreadedExecutor
from controller import step_controller, build_in_pose
from controller.controller_client import ControllerClient
from arm_kinematics.kinematics_control import set_pose_target
from servo_controller.bus_servo_control import set_servo_position
from arm_kinematics_msgs.srv import SetRobotPose
from arm_kinematics import transform
from servo_controller_msgs.msg import ServosPosition



class PerformActions(Node):
    def __init__(self):
        super().__init__('perform_actions')
        self.z_dis = 0.30
        self.x_init = transform.link3 + transform.tool_link
        self._action_lock = threading.Lock()
        self._is_running = False
        self._should_stop = False
        self._action_thread = None # 用来持有当前动作线程的引用

        self.joints_pub = self.create_publisher(ServosPosition, '/servo_controller', 1) # 舵机控制(servo control)

        self.create_subscription(RunActionSet, '~/actions', self.actions_callback,  1 )
        self.step_controller = step_controller.StepController()
        self.controller = ControllerClient()
        self.arm_kinematics_client = self.create_client(SetRobotPose, '/arm_kinematics/set_pose_target')
        self.arm_kinematics_client.wait_for_service()
        
    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def actions_callback(self, msg: RunActionSet):
        action_name = msg.action_path
        
        if action_name == 'stop':
            self.get_logger().info("Received stop command.")
            self.stop_current_action()
        else:
            self.get_logger().info(f"Received command to start action: {action_name}")
            self.start_action(action_name)

    def start_action(self, action_name):
        with self._action_lock:
            if self._is_running:
                self.get_logger().warn("An action is already running. Please stop it first.")
                return

        # --- 3. 创建并启动一个后台线程来执行耗时动作 ---
        # 根据动作名称选择要执行的函数
        if action_name == 'twist':
            target_func = self.twist # 要执行的函数是 wave
        elif action_name == 'turn_round':
            target_func = self.turn_round # 可以扩展其他动作
        elif action_name == 'square':
            target_func = self.dance_routine # 可以扩展其他动作
        elif action_name == 'robot_showtime':
            target_func = self.robot_showtime # 可以扩展其他动作
        else:
            self.get_logger().error(f"Unknown action name: {action_name}")
            return
            
        self._action_thread = threading.Thread(target=target_func)
        self._action_thread.daemon = True
        self._action_thread.start()

    def stop_current_action(self):
        # 设置停止标志，后台线程会检测到这个标志并自行退出
        self._should_stop = True
        self.get_logger().info("Stop flag set. Waiting for action thread to terminate...")
        # (可选) 可以等待线程结束
        if self._action_thread and self._action_thread.is_alive():
             self._action_thread.join(timeout=1.0) # 等待1秒
        self.get_logger().info("Action stopped.")

    # --- 4. 辅助函数：可中断的睡眠 ---
    def _interruptible_sleep(self, duration):
        start_time = time.time()
        while time.time() - start_time < duration:
            if self._should_stop:
                return True # 被中断
            time.sleep(0.02) # 短暂睡眠并检查
        return False # 正常结束

    def twist(self):
        """
        机体扭动（非阻塞、可中断版本）
        """
        # --- 5. 在动作开始和结束时，正确设置状态标志 ---
        with self._action_lock:
            self._is_running = True
        self._should_stop = False # 重置停止标志
        self.get_logger().info("Wave action started.")

        try:
            duration = 0.03
            org_pose = tuple(build_in_pose.DEFAULT_POSE_M)
            
            # --- 6. 将所有 time.sleep 替换为 _interruptible_sleep ---
            # --- 并在关键位置检查 self._should_stop ---
            if self._interruptible_sleep(0.8): return # 如果被中断，直接返回
            
            # 逐渐加快并加大摇摆幅度
            for j in range(7, 16, 2):
                if self._should_stop: return
                i = 90 
                j = min(15, j)
                while i <= 360 + 90:
                    if self._should_stop: return
                    if i == 90 and j == 7:
                        t = 0.5
                    else:
                        t = duration
                    i += 4 + j * 0.30
                    x = math.sin(math.radians(i)) * (0.018 * (j + ((i - 90) / 360) * 2))
                    y = math.cos(math.radians(i)) * (0.018 * (j + ((i - 90) / 360) * 2))
                    pose = kinematics_calculate.transform_euler(org_pose, (0, 0, 0), 'xy', (x, y), degrees=False)
                    
                    # 假设 set_pose_base 是非阻塞的
                    self.step_controller.set_pose_base(pose, t) 
                    
                    if self._interruptible_sleep(t): return

            # ... (省略第二个循环，修改方法完全相同) ...
            for j in range(16, 7, -2):
                if self._should_stop: return
                i = 360 + 90
                while i >= 90 :
                    if self._should_stop: return

                    if i == 90 and j == 7:
                        t = 0.5
                    else:
                        t = duration
                    i += -(4 + j * 0.30)
                    k = 360 + 90 - i + 90
                    x = math.sin(math.radians(i)) * (0.018 * (j + (1 - (i - 90) / 360) * -2))
                    y = math.cos(math.radians(i)) * (0.018 * (j + (1 - (i - 90) / 360) * -2))
                    pose = kinematics_calculate.transform_euler(org_pose, (0, 0, 0), 'xy', (x, y), degrees=False)
                    
                    # 假设 set_pose_base 是非阻塞的
                    self.step_controller.set_pose_base(pose, t) 
                    
                    if self._interruptible_sleep(t): return


            if self._should_stop: return
            self.step_controller.set_build_in_pose('DEFAULT_POSE_M', 1)
            if self._interruptible_sleep(1): return
            
            self.get_logger().info("Wave action finished successfully.")

        finally:
            # --- 7. 无论如何，最后都要重置状态标志 ---
            with self._action_lock:
                self._is_running = False
            self._should_stop = False
            self.get_logger().info("Action state has been reset.")

    def turn_round(self):
        """
        表演模式的虚拟动作组， 扭身(the performance mode of the virtual action group, twist body)
        """
        self.step_controller.set_build_in_pose('DEFAULT_POSE_M', 0.8)
        # self.set_pose_base(build_in_pose.DEFAULT_POSE_M, 0.8)
        org_pose = tuple(build_in_pose.DEFAULT_POSE_M)
        time.sleep(0.8)
        pose = kinematics_calculate.transform_euler(org_pose, (0, 0, 30), 'xyz', (0, 0, 0.8), degrees=False)
        print(pose)
        self.step_controller.set_pose_base(pose, 0.3)
        for i in range(0, 7):
            pose = kinematics_calculate.transform_euler(org_pose, (0, 0, -30), 'xyz', (0, 0, -0.8), degrees=False)
            self.step_controller.set_pose_base(pose, 0.6)
            time.sleep(0.6)
            pose = kinematics_calculate.transform_euler(org_pose, (0, 0, 30), 'xyz', (0, 0, 0.8), degrees=False)
            self.step_controller.set_pose_base(pose, 0.6)
            time.sleep(0.6)
        self.step_controller.set_pose_base(build_in_pose.DEFAULT_POSE_M, 1)

    def dance_routine(self):
        """
        一段富有表演性的舞蹈程序（可中断）
        """
        # --- 同样，状态管理应该在更高层完成 ---
        self.get_logger().info("Dance Routine Started!")

        try:
            # --- 辅助函数，和上面 square 函数中的一样 ---
            def interruptible_sleep(duration):
                # ... (代码同上) ...
                start_time = time.time()
                while time.time() - start_time < duration:
                    if self._should_stop: return True
                    time.sleep(0.02)
                return False

            def execute_traveling_and_wait(gait, stride, height, direction, rotation, period, steps):
                # ... (代码同上) ...
                self.step_controller.set_step_mode(gait, stride, height, direction, rotation, period, steps)
                self.step_controller.stopped.clear()
                while not self.step_controller.stopped.is_set():
                    if self._should_stop:
                        self.step_controller.stop_running()
                        return True
                    time.sleep(0.1)
                return False

            # --- 舞蹈流程开始 ---

            # 1. 准备动作：回到默认姿态
            self.step_controller.set_build_in_pose('DEFAULT_POSE', 1.0)
            if interruptible_sleep(1.2): return

            # 2. 点头致意：身体前倾再恢复
            self.get_logger().info("Dance: Nodding...")
            # 向前倾斜约15度
            self.step_controller.transform_pose_euler(translate=(0, 0, 0), axis='xyz', euler=(15, 0, 0), duration=0.8)
            if interruptible_sleep(1.0): return
            # 恢复
            self.step_controller.transform_pose_euler(translate=(0, 0, 0), axis='xyz', euler=(-15, 0, 0), duration=0.8)
            if interruptible_sleep(1.0): return
            # 3. 扭腰热身：调用我们之前写的 wave 动作
            # 注意：这里需要将 wave 函数也改造成非阻塞的，或者直接在这里实现一个简化版
            self.get_logger().info("Dance: Waving body...")
            # 假设有一个简化版的 wave 函数可以调用
            if self.simple_wave(): return # simple_wave 内部需要检查 self._should_stop

            # 4. 菱形舞步：走一个45度的菱形
            self.get_logger().info("Dance: Diamond Step!")
            steps = 4
            step_time = 0.5
            # 右前45度
            if execute_traveling_and_wait(1, 35, 15, math.radians(45), 0, step_time, steps): return
            # 右后45度
            if execute_traveling_and_wait(1, 35, 15, math.radians(-45), 0, step_time, steps): return
            # 左后45度
            if execute_traveling_and_wait(1, 35, 15, math.radians(-135), 0, step_time, steps): return
            # 左前45度
            if execute_traveling_and_wait(1, 35, 15, math.radians(135), 0, step_time, steps): return

            # 5. 再次扭腰庆祝
            self.get_logger().info("Dance: Final Wave!")
            if self.simple_wave(): return

            # 6. 鞠躬致谢：身体大幅度前倾
            self.get_logger().info("Dance: Taking a bow!")
            self.step_controller.transform_pose_euler(translate=(0, 0, -35), axis='xyz', euler=(0, 0.5, 0), duration=1.5)
            if interruptible_sleep(1.7): return
            
            # 7. 结束动作：缓慢恢复
            self.step_controller.set_build_in_pose('DEFAULT_POSE', 2.0)
            if interruptible_sleep(2.2): return

            self.get_logger().info("Dance Routine Finished!")

        finally:
            # 确保最后停下来
            self.step_controller.stop_running()
            self.step_controller.set_build_in_pose('DEFAULT_POSE', 1.0)

    def simple_wave(self):
        # 这是一个简化的、可中断的 wave 版本
        # 假设 self.step_controller 有一个 set_pose_base
        org_pose = self.step_controller.pose
        for _ in range(2): # 扭两次
            for angle in [-15, 15, 0]: # 左右摇摆
                if self._should_stop: return True
                # 使用 transform_pose_euler 来实现身体侧倾
                # 绕 X 轴旋转 (roll)
                self.step_controller.transform_pose_euler(translate=(0,0,0), axis='xyz', euler=(math.radians(angle), 0, 0), duration=0.4)
                
                # 等待动作完成
                start_time = time.time()
                while time.time() - start_time < 0.5:
                    if self._should_stop: return True
                    time.sleep(0.02)
        return False
    # 假设这个函数在您的 PerformActions 类中

    def robot_showtime(self):
        """
        一个全新的、富有表现力的机器人舞蹈表演（可中断）。
        A brand-new, expressive robot dance performance (interruptible).
        """
        self.get_logger().info("Showtime is beginning!")

        # ------------------ 辅助函数 (与之前相同) ------------------
        def interruptible_sleep(duration):
            start_time = time.time()
            while time.time() - start_time < duration:
                if self._should_stop: return True
                time.sleep(0.02)
            return False

        def execute_traveling_and_wait(gait, stride, height, direction, rotation, period, steps):
            self.step_controller.set_step_mode(gait, stride, height, direction, rotation, period, steps)
            self.step_controller.stopped.clear()
            while not self.step_controller.stopped.is_set():
                if self._should_stop:
                    self.step_controller.stop_running()
                    return True
                time.sleep(0.1)
            return False

        # ------------------ 舞蹈正式开始 ------------------
        try:
            # ==================== 1. 开场 (Opening) ====================
            
            # 准备姿态
            self.step_controller.set_build_in_pose('DEFAULT_POSE', 1.5)
            # 机械臂初始姿态：收回到胸前
            msg = set_pose_target([self.x_init, 0.0, self.z_dis], 0.0, [-180.0, 180.0], 1.5)
            res = self.send_request(self.arm_kinematics_client, msg)
            if res.pulse:
                set_servo_position(self.joints_pub, 1.5, ((24, 500), (23, 500), (22, res.pulse[3]), (21, res.pulse[2]), (20, res.pulse[1]), (19, res.pulse[0])))
            if interruptible_sleep(1.7): return
            self.get_logger().info("Showtime: 原地踏步...")
            if execute_traveling_and_wait(gait=2, stride=0, height=15, direction=0, rotation=0.3, period=0.0, steps=4): return 
            
            self.get_logger().info("Showtime: 左转...")
            self.step_controller.transform_pose_euler(translate=(0, 0, 0), axis='xyz', euler=(0, 0,  math.radians(45)), duration=0.6)
            set_servo_position(self.joints_pub, 0.6, ((19, 750),))
            if interruptible_sleep(1.0): return
            
            self.get_logger().info("Showtime: 原地踏步+机械臂上下动...")

            if execute_traveling_and_wait(gait=2, stride=0, height=15, direction=0, rotation=0.3, period=0.0, steps=4): return 
            for _ in range(4):
                msg = set_pose_target([self.x_init, 0.0, self.z_dis + 0.05], 0.0, [-180.0, 180.0], 0.5)
                res = self.send_request(self.arm_kinematics_client, msg)
                if res.pulse:
                    set_servo_position(self.joints_pub, 1.5, ((24, 500), (23, 500), (22, res.pulse[3]), (21, res.pulse[2]), (20, res.pulse[1]), (19, res.pulse[0])))
                if interruptible_sleep(0.5): return

                msg = set_pose_target([self.x_init, 0.0, self.z_dis - 0.05], 0.0, [-180.0, 180.0], 0.5)
                res = self.send_request(self.arm_kinematics_client, msg)
                if res.pulse:
                    set_servo_position(self.joints_pub, 1.5, ((24, 500), (23, 500), (22, res.pulse[3]), (21, res.pulse[2]), (20, res.pulse[1]), (19, res.pulse[0])))
                if interruptible_sleep(0.5): return


            # "唤醒"动作：身体轻微起伏两次
            # self.get_logger().info("Showtime: Waking up...")
            # for _ in range(2):
            #     self.step_controller.transform_pose_euler(translate=(0, 0, 30), axis='xyz', euler=(0, 0, 0), duration=0.6)
            #     if interruptible_sleep(0.5): return
            #     self.step_controller.transform_pose_euler(translate=(0, 0, -30), axis='xyz', euler=(0, 0, 0), duration=0.6)
            #     if interruptible_sleep(0.5): return

            # ==================== 2. 发展 (Development) ====================

            # "环顾四周"：身体左右旋转，配合机械臂小范围挥舞
            self.get_logger().info("Showtime: Looking around...")
            for angle in [20, -20, 0]: # 左看，右看，回正
                # 身体旋转
                self.step_controller.transform_pose_euler(translate=(0, 0, 0), axis='xyz', euler=(0, 0, math.radians(angle)), duration=1.0)
                # 机械臂向同方向小幅摆动
                arm_x = self.x_init + abs(angle) # 稍微伸出
                arm_y = 0.4 * np.sign(angle) if angle != 0 else 0 # 左右摆动
                msg = set_pose_target([arm_x, arm_y, 0.3], 0.0, [-180.0, 180.0], 1.0)
                res = self.send_request(self.arm_kinematics_client, msg)
                if res.pulse:
                    set_servo_position(self.joints_pub, 1.0, ((24, 500), (23, 500), (22, res.pulse[3]), (21, res.pulse[2]), (20, res.pulse[1]), (19, res.pulse[0])))
                if interruptible_sleep(1.2): return

            # 轻快的踏步舞：原地踏步，身体左右摇摆
            self.get_logger().info("Showtime: Happy feet!")
            # 启动一个后台线程来执行踏步
            # traveling_thread = threading.Thread(target=execute_traveling_and_wait, args=(2, 0, 25, 0, 0, 0.3, 0)) # repeat=0 表示一直走
            # traveling_thread.daemon = True
            # traveling_thread.start()
            
            # 在踏步的同时，身体左右摇摆
            for _ in range(4): # 摇摆4次
                self.step_controller.transform_pose_euler(translate=(0, 20, 0), axis='xyz', euler=(math.radians(10), 0, 0), duration=0.4)
                if interruptible_sleep(0.4): break
                self.step_controller.transform_pose_euler(translate=(0, -20, 0), axis='xyz', euler=(math.radians(-10), 0, 0), duration=0.4)
                if interruptible_sleep(0.4): break
            
            # 停止踏步
            # self.step_controller.stop_running()
            # if interruptible_sleep(0.5): return
            
            # ==================== 3. 高潮 (Climax) ====================

            # 大风车：身体快速自转，配合机械臂展开
            self.get_logger().info("Showtime: The Windmill!")
            # 机械臂向上伸展
            msg = set_pose_target([0.8, 0.0, 0.5], 0.0, [-180.0, 180.0], 1.5)
            res = self.send_request(self.arm_kinematics_client, msg)
            if res.pulse:
                set_servo_position(self.joints_pub, 1.5, ((24, 500), (23, 500), (22, res.pulse[3]), (21, res.pulse[2]), (20, res.pulse[1]), (19, res.pulse[0])))
            if interruptible_sleep(1.6): return

            # 旋转！使用 set_step_mode 的 rotation 参数
            if execute_traveling_and_wait(gait=2, stride=0, height=15, direction=0, rotation=0.3, period=0.6, steps=10): return # rotation=0.8 表示较快的旋转速度

            # ==================== 4. 结尾 (Finale) ====================

            # 慢速恢复：旋转减速停止
            self.get_logger().info("Showtime: Cooling down...")
            if execute_traveling_and_wait(gait=2, stride=0, height=10, direction=0, rotation=0.1, period=1.0, steps=3): return
            
            # 鞠躬致谢
            self.get_logger().info("Showtime: Taking a bow!")
            # 身体前倾
            self.step_controller.transform_pose_euler(translate=(0, 0, -30), axis='xyz', euler=(0, 0.4, 0), duration=2.0)
            # 机械臂向前伸出，做出“谢谢”的手势
            msg = set_pose_target([self.x_init+0.1, 0.0, self.z_dis], 0.0, [-180.0, 180.0], 1.5)
            res = self.send_request(self.arm_kinematics_client, msg)
            if res.pulse:
                # 爪子张开，像在挥手
                set_servo_position(self.joints_pub, 2.0, ((24, 800), (23, 500), (22, res.pulse[3]), (21, res.pulse[2]), (20, res.pulse[1]), (19, res.pulse[0])))
            if interruptible_sleep(2.5): return

            # 最终姿态：缓慢恢复
            self.get_logger().info("Showtime: Finished!")
            self.step_controller.set_build_in_pose('DEFAULT_POSE', 2.5)
            msg = set_pose_target([self.x_init, 0.0, self.z_dis], 0.0, [-180.0, 180.0], 1.5)
            res = self.send_request(self.arm_kinematics_client, msg)
            if res.pulse:
                set_servo_position(self.joints_pub, 2.5, ((24, 500), (23, 500), (22, res.pulse[3]), (21, res.pulse[2]), (20, res.pulse[1]), (19, res.pulse[0])))
            if interruptible_sleep(2.8): return

        finally:
            # 最后的保险措施
            self.get_logger().info("Ensuring all movements are stopped.")
            self.step_controller.stop_running()
            self.step_controller.set_build_in_pose('DEFAULT_POSE', 1.0)

def main():
    rclpy.init()
    # 注意：PerformActions 内部创建了 StepController，这两个都是Node
    # 我们需要把它们都加入到执行器中
    node = PerformActions()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    if hasattr(node, 'step_controller') and isinstance(node.step_controller, Node):
        executor.add_node(node.step_controller)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if hasattr(node, 'step_controller') and isinstance(node.step_controller, Node):
            node.step_controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()