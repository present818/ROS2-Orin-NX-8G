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
        elif action_name == 'dance':
            target_func = self.dance # 可以扩展其他动作
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
    def _perform_arm_waving_1(self, repetitions: int, arm_joint_id: int, arm_target_pulse: int):
        """
        一个辅助函数，在后台执行机械臂的上下挥舞动作。
        An auxiliary function to perform the arm's up-and-down waving motion in the background.
        """
        self.get_logger().info("Arm waving thread started.")
        try:
            moves = [
                (0.03, 500),
                (-0.03, 400),
                (-0.03, 500),
                (0.03, 600),
                (0.03, 500),
                (-0.03, 600),
                (-0.03, 500),
                (0.03, 400),
                (0.0, 500),
                ]
            for move in moves:
                dis, pulse = move
                
                msg = set_pose_target([self.x_init, 0.0, self.z_dis + dis], 0.0, [-180.0, 180.0], 1.0)
                res = self.send_request(self.arm_kinematics_client, msg)
                if res and res.pulse:
                    set_servo_position(self.joints_pub, 0.5, ((24, 500), (23, 500), (22, res.pulse[3]), (21, res.pulse[2]), (20, res.pulse[1]), (19, pulse)))
                
                # 使用可中断的睡眠
                if self._interruptible_sleep(0.5): return

                # 检查是否需要提前停止
                if self._should_stop:
                    self.get_logger().info("Arm waving interrupted.")
                    break
        finally:
            self.get_logger().info("Arm waving thread finished.")

    def _perform_arm_waving(self, repetitions: int, arm_joint_id: int, arm_target_pulse: int):
        """
        一个辅助函数，在后台执行机械臂的上下挥舞动作。
        An auxiliary function to perform the arm's up-and-down waving motion in the background.
        """
        self.get_logger().info("Arm waving thread started.")
        try:
            for i in range(repetitions):
                # 检查是否需要提前停止
                if self._should_stop:
                    self.get_logger().info("Arm waving interrupted.")
                    break

                self.get_logger().debug(f"Arm wave cycle {i+1}/{repetitions}: Moving up.")
                msg = set_pose_target([self.x_init, 0.0, self.z_dis + 0.03], 0.0, [-180.0, 180.0], 1.0)
                res = self.send_request(self.arm_kinematics_client, msg)
                if res and res.pulse:
                    set_servo_position(self.joints_pub, 0.5, ((24, 500), (23, 500), (22, res.pulse[3]), (21, res.pulse[2]), (20, res.pulse[1]), (arm_joint_id, arm_target_pulse)))
                
                # 使用可中断的睡眠
                if self._interruptible_sleep(0.5): return

                # 检查是否需要提前停止
                if self._should_stop:
                    self.get_logger().info("Arm waving interrupted.")
                    break

                self.get_logger().debug(f"Arm wave cycle {i+1}/{repetitions}: Moving down.")
                msg = set_pose_target([self.x_init, 0.0, self.z_dis - 0.03], 0.0, [-180.0, 180.0], 1.0)
                res = self.send_request(self.arm_kinematics_client, msg)
                if res and res.pulse:
                    set_servo_position(self.joints_pub, 0.5, ((24, 500), (23, 500), (22, res.pulse[3]), (21, res.pulse[2]), (20, res.pulse[1]), (arm_joint_id, arm_target_pulse)))

                if self._interruptible_sleep(0.5): return

        finally:
            self.get_logger().info("Arm waving thread finished.")

    def _perform_arm_sweep(self, start_y, end_y, duration_secs):
        """
        辅助函数：在指定时间内平滑地移动机械臂的Y轴。
        Helper function: Smoothly moves the arm's Y-axis over a specified duration.
        """
        self.get_logger().info(f"Arm sweep started from y={start_y} to y={end_y}.")
        start_time = time.time()
        
        # 使用 numpy.linspace 可以生成平滑的中间点
        # 注意：你的 for i in range(0, 0.2, 0.05) 有语法问题，range()不支持浮点数步长。
        # 我们用 np.linspace 来实现这个效果，假设移动20个步骤。
        positions = np.linspace(start_y, end_y, num=20)
        step_duration = duration_secs / len(positions)

        try:
            for y_pos in positions:
                if self._should_stop:
                    self.get_logger().info("Arm sweep interrupted.")
                    return

                # 发送机械臂位置指令
                msg = set_pose_target([self.x_init, y_pos, self.z_dis], 0.0, [-180.0, 180.0], 1.0)
                res = self.send_request(self.arm_kinematics_client, msg)
                if res and res.pulse:
                    # 使用一个较短的时间来平滑过渡，而不是阻塞整个流程
                    set_servo_position(self.joints_pub, step_duration, ((24, 500), (23, 500), (22, res.pulse[3]), (21, res.pulse[2]), (20, res.pulse[1]), (19, res.pulse[0])))
                
                # 使用可中断的短睡眠等待舵机指令发出和执行
                if self._interruptible_sleep(step_duration):
                    return
        finally:
            self.get_logger().info("Arm sweep finished.")

    def gen_circle(self, r, start_angle_deg=180, reverse=False):
        """
        生成一个平滑的、从指定角度开始的圆周路径点。

        :param r: 圆的半径 (单位是毫米)。
        :param start_angle_deg: 路径的起始角度 (度)。
        :param reverse: 是否反向生成路径点。
        :return: 一个包含 (x, y) 坐标元组的列表。
        """
        points = []
        # 生成 0 到 355 度的角度
        angles = np.arange(0, 360, 5)

        # 找到起始角度在数组中的索引
        start_index = int(start_angle_deg / 5) % len(angles)

        # 重新排列角度数组，让起始角度在最前面
        ordered_angles_deg = np.roll(angles, -start_index)

        if reverse:
            # 反转路径，同时保持起点不变
            ordered_angles_deg = np.concatenate(([ordered_angles_deg[0]], np.flip(ordered_angles_deg[1:])))

        for angle_deg in ordered_angles_deg:
            rad = math.radians(angle_deg)
            x = r * math.cos(rad)
            y = r * math.sin(rad)
            points.append((x, y))
            
        # 添加最后一个点，与起点重合，形成完美闭环
        points.append(points[0])
        
        return points


    def wait_for_action(self, duration):
        """
        等待指定的时长，同时以高频率检查是否需要停止。
        """
        end_time = time.time() + duration
        while time.time() < end_time:
            if self._should_stop:
                return True  # 被中断
            # time.sleep(0.02) # 短暂休眠，避免CPU空转，并让出时间给其他线程

        return False # 正常完成

    def _perform_arm_path(self, points_mm, duration_secs):
        """
        辅助函数：让机械臂跟随给定的路径点列表运动 (修正了时间控制)。
        """
        self.get_logger().info(f"Arm path motion started with {len(points_mm)} points.")
        
        num_points = len(points_mm)
        if num_points <= 1:
            return
            
        path_to_traverse = points_mm[1:]
        step_duration = duration_secs / len(path_to_traverse)

        try:
            for x_mm, y_mm in path_to_traverse:
                if self._should_stop:
                    self.get_logger().info("Arm path motion interrupted.")
                    return

                # 计算并发送舵机指令
                target_x = self.x_init + (x_mm / 1000.0)
                target_y = y_mm / 1000.0
                msg = set_pose_target([target_x, target_y, self.z_dis], 0.0, [-180.0, 180.0], 1.0)
                res = self.send_request(self.arm_kinematics_client, msg)
                if res and res.pulse:
                    # 命令舵机用 step_duration 的时间移动
                    set_servo_position(self.joints_pub, step_duration, 
                                    ((24, 500), (23, 500), 
                                        (22, res.pulse[3]), (21, res.pulse[2]), 
                                        (20, res.pulse[1]), (19, res.pulse[0])))
                
                # 【关键改动】
                # 我们不再使用 interruptible_sleep，而是用新的函数等待，
                # 这个等待与舵机的物理移动时间是并行的。
                if self.wait_for_action(step_duration):
                    return
        finally:
            self.get_logger().info("Arm path motion finished.")


    def _perform_arm_reach(self, start_x_offset, end_x_offset, duration_secs):
        """
        辅助函数：在指定时间内，平滑地移动机械臂的X轴（前后伸缩）。
        x_offset 是相对于 self.x_init 的偏移量。
        """
        self.get_logger().info(f"Arm reach started from x_offset={start_x_offset} to {end_x_offset}.")
        
        # 使用 numpy.linspace 生成平滑的中间点
        positions = np.linspace(start_x_offset, end_x_offset, num=20)
        
        # 检查 duration_secs 是否有效，避免除零错误
        if len(positions) > 0:
            step_duration = duration_secs / len(positions)
        else:
            return

        try:
            for x_offset in positions:
                if self._should_stop:
                    self.get_logger().info("Arm reach interrupted.")
                    return

                # 计算机械臂的绝对X坐标
                target_x = self.x_init + x_offset
                
                # 发送机械臂位置指令 (Y和Z轴保持不变)
                msg = set_pose_target([target_x, 0.0, self.z_dis], 0.0, [-180.0, 180.0], 1.0)
                res = self.send_request(self.arm_kinematics_client, msg)
                if res and res.pulse:
                    set_servo_position(self.joints_pub, step_duration, 
                                    ((24, 500), (23, 500), 
                                        (22, res.pulse[3]), (21, res.pulse[2]), 
                                        (20, res.pulse[1]), (19, res.pulse[0])))
                
                # 使用可中断的等待
                if self._interruptible_sleep(step_duration):
                    return
        finally:
            self.get_logger().info("Arm reach finished.")
            
    def _perform_arm_nod(self, repetitions, duration_per_nod):
        """
        辅助函数：让机械臂执行快速的点头动作。
        """
        self.get_logger().info(f"Arm nodding action started for {repetitions} reps.")
        
        original_z = self.z_dis
        nod_down_z = original_z - 0.04 # 点头时向下移动4厘米

        try:
            for _ in range(repetitions):
                if self._should_stop:
                    self.get_logger().info("Arm nodding interrupted.")
                    return

                # 1. 向下点头
                msg = set_pose_target([self.x_init, 0.0, nod_down_z], 0.0, [-180.0, 180.0], 1.0)
                res = self.send_request(self.arm_kinematics_client, msg)
                if res and res.pulse:
                    set_servo_position(self.joints_pub, duration_per_nod / 2, 
                                    ((24, 500), (23, 500), 
                                        (22, res.pulse[3]), (21, res.pulse[2]), 
                                        (20, res.pulse[1]), (19, res.pulse[0])))
                
                if self._interruptible_sleep(duration_per_nod / 2): return

                # 2. 恢复到原位
                if self._should_stop: return
                msg = set_pose_target([self.x_init, 0.0, original_z], 0.0, [-180.0, 180.0], 1.0)
                res = self.send_request(self.arm_kinematics_client, msg)
                if res and res.pulse:
                    set_servo_position(self.joints_pub, duration_per_nod / 2, 
                                    ((24, 500), (23, 500), 
                                        (22, res.pulse[3]), (21, res.pulse[2]), 
                                        (20, res.pulse[1]), (19, res.pulse[0])))

                if self._interruptible_sleep(duration_per_nod / 2): return

        finally:
            # 确保最后恢复到初始位置
            msg = set_pose_target([self.x_init, 0.0, original_z], 0.0, [-180.0, 180.0], 1.0)
            res = self.send_request(self.arm_kinematics_client, msg)
            if res and res.pulse:
                set_servo_position(self.joints_pub, 0.3, ((24, 500), (23, 500), (22, res.pulse[3]), (21, res.pulse[2]), (20, res.pulse[1]), (19, res.pulse[0])))
            self.get_logger().info("Arm nodding action finished.")

    def _perform_arm_reach(self, start_x_offset, end_x_offset, duration_secs):
        """
        辅助函数：在指定时间内，平滑地移动机械臂的X轴（前后伸缩）。
        x_offset 是相对于 self.x_init 的偏移量。
        """
        self.get_logger().info(f"Arm reach started from x_offset={start_x_offset} to {end_x_offset}.")
        
        # 使用 numpy.linspace 生成平滑的中间点
        positions = np.linspace(start_x_offset, end_x_offset, num=20)
        
        # 检查 duration_secs 是否有效，避免除零错误
        if len(positions) > 0:
            step_duration = duration_secs / len(positions)
        else:
            return

        try:
            for x_offset in positions:
                if self._should_stop:
                    self.get_logger().info("Arm reach interrupted.")
                    return

                # 计算机械臂的绝对X坐标
                target_x = self.x_init + x_offset
                
                # 发送机械臂位置指令 (Y和Z轴保持不变)
                msg = set_pose_target([target_x, 0.0, self.z_dis], 0.0, [-180.0, 180.0], 1.0)
                res = self.send_request(self.arm_kinematics_client, msg)
                if res and res.pulse:
                    set_servo_position(self.joints_pub, step_duration, 
                                    ((24, 500), (23, 500), 
                                        (22, res.pulse[3]), (21, res.pulse[2]), 
                                        (20, res.pulse[1]), (19, res.pulse[0])))
                
                # 使用可中断的等待
                if self._interruptible_sleep(step_duration):
                    return
        finally:
            self.get_logger().info("Arm reach finished.")
            
    def dance(self):
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
            # msg = set_pose_target([self.x_init, 0.0, self.z_dis], 0.0, [-180.0, 180.0], 1.0)
            # res = self.send_request(self.arm_kinematics_client, msg)
            # if res.pulse:
            #     set_servo_position(self.joints_pub, 1.0, ((24, 500), (23, 500), (22, res.pulse[3]), (21, res.pulse[2]), (20, res.pulse[1]), (19, res.pulse[0])))
            # if interruptible_sleep(1.0): return
            # self.get_logger().info("Showtime: 原地踏步...")
            # if execute_traveling_and_wait(gait=2, stride=0, height=15, direction=0, rotation=0, period=0.5, steps=2): return 
            
            # self.get_logger().info("Showtime: 左转...")
            # self.step_controller.transform_pose_euler(translate=(0, 0, 0), axis='xyz', euler=(0, 0, 15), duration=0.5)
            # set_servo_position(self.joints_pub, 0.5, ((19, 750),))
            # if interruptible_sleep(0.5): return
            
            # # self.get_logger().info("Showtime: 原地踏步+机械臂上下动...")

            # # self.get_logger().info("Showtime: Stepping in place while waving left arm...")
            
            # # 1. 启动踏步（非阻塞），steps=0 表示无限循环，直到被手动停止
            # self.step_controller.set_step_mode(2, 0, 15, 0, 0, 0.5, 0)
            # self.step_controller.stopped.clear()

            # # 2. 在新线程中执行机械臂挥舞
            # #    这里的参数 (4, 19, 750) 对应 (重复次数, 舵机ID, 舵机脉冲值)
            # arm_thread_1 = threading.Thread(target=self._perform_arm_waving, args=(2, 19, 650))
            # arm_thread_1.daemon = True
            # arm_thread_1.start()

            # # 3. 等待机械臂动作完成
            # arm_thread_1.join()

            # # 4. 停止踏步
            # self.get_logger().info("Arm waving finished, stopping stepping...")
            # self.step_controller.stop_running()
            # # 确保踏步完全停止后再继续
            # while not self.step_controller.stopped.is_set():
            #     if self._should_stop: return # 如果在等待时收到停止指令，则退出
            #     time.sleep(0.1)
            
            # # 恢复机械臂初始姿态
            # set_servo_position(self.joints_pub, 0.5, ((19, 500),))
            # # if interruptible_sleep(1.0): return
            # # ------------------- 结束并行执行区域 1 (左侧) -------------------

            # self.get_logger().info("Showtime: 右转...")
            # self.step_controller.transform_pose_euler(translate=(0, 0, 0), axis='xyz', euler=(0, 0,  -15), duration=0.5) # 改为-90度，这样左右对称
            # if interruptible_sleep(0.5): return

            # # ------------------- 开始并行执行区域 2 (右侧) -------------------
            # self.get_logger().info("Showtime: Stepping in place while waving right arm...")
            
            # # 1. 再次启动踏步
            # self.step_controller.set_step_mode(2, 0, 15, 0, 0, 0.5, 0)
            # self.step_controller.stopped.clear()

            # # 2. 在新线程中执行机械臂挥舞 (这次是右臂，舵机ID 19, 脉冲 250)
            # arm_thread_2 = threading.Thread(target=self._perform_arm_waving, args=(2, 19, 350))
            # arm_thread_2.daemon = True
            # arm_thread_2.start()

            # self.step_controller.transform_pose_euler(translate=(0, 0, 0), axis='xyz', euler=(0, 0,  -15), duration=0.5) # 改为-90度，这样左右对称
            # if interruptible_sleep(0.5): return
            # # 3. 等待机械臂动作完成
            # arm_thread_2.join()

            # # 4. 停止踏步
            # self.get_logger().info("Arm waving finished, stopping stepping...")
            # self.step_controller.stop_running()
            # while not self.step_controller.stopped.is_set():
            #     if self._should_stop: return
            #     time.sleep(0.1)

            # # 恢复机械臂初始姿态
            # set_servo_position(self.joints_pub, 0.5, ((19, 500),))
            # self.step_controller.set_build_in_pose('DEFAULT_POSE', 0.5)
            # if interruptible_sleep(0.5): return

 
            # # ==================== 2. 平移动作与机械臂协同 (Parallel Moves) ====================

            # --- 动作一：向左平移，同时机械臂向左挥出 ---
            self.get_logger().info("Showtime: Translating left with arm sweep...")
            # 定义平移参数
            travel_period = 0.5
            travel_steps = 4
            travel_duration = travel_period * travel_steps # 总计 2.0 秒

            # 1. 在新线程中启动机械臂平扫动作
            arm_sweep_left_thread = threading.Thread(target=self._perform_arm_sweep, args=(0.0, 0.15, travel_duration))
            arm_sweep_left_thread.daemon = True
            arm_sweep_left_thread.start()
            
            # 2. 主线程执行身体平移动作，并等待其完成
            if execute_traveling_and_wait(gait=2, stride=25, height=15, direction=math.radians(90), rotation=0, period=travel_period, steps=travel_steps): return
            if execute_traveling_and_wait(gait=2, stride=25, height=15, direction=math.radians(90), rotation=0, period=travel_period, steps=travel_steps): return

            # 3. 确保机械臂线程也已结束
            arm_sweep_left_thread.join()

            # --- 动作二：向右平移，同时机械臂收回并向右挥出 ---
            self.get_logger().info("Showtime: Translating right with arm sweep...")
            # 定义平移参数（向右走8步，这样动作更完整）
            travel_steps = 8
            travel_duration = travel_period * travel_steps # 总计 4.0 秒

            # 1. 在新线程中启动机械臂从左到右的大范围平扫
            arm_sweep_right_thread = threading.Thread(target=self._perform_arm_sweep, args=(0.15, -0.15, travel_duration))
            arm_sweep_right_thread.daemon = True
            arm_sweep_right_thread.start()

            # 2. 主线程执行身体向右平移
            if execute_traveling_and_wait(gait=2, stride=25, height=15, direction=math.radians(270), rotation=0, period=travel_period, steps=travel_steps): return
            
            # 3. 确保机械臂线程也已结束
            arm_sweep_right_thread.join()

            # --- 动作三：向左平移，同时机械臂收回到中间 ---
            self.get_logger().info("Showtime: Translating left to center arm...")
            travel_steps = 4
            travel_duration = travel_period * travel_steps # 总计 2.0 秒

            # 1. 启动机械臂收回动作
            arm_sweep_center_thread = threading.Thread(target=self._perform_arm_sweep, args=(-0.15, 0.0, travel_duration))
            arm_sweep_center_thread.daemon = True
            arm_sweep_center_thread.start()
            
            # 2. 主线程执行身体向左平移回到中心
            if execute_traveling_and_wait(gait=2, stride=25, height=15, direction=math.radians(90), rotation=0, period=travel_period, steps=travel_steps): return
            
            # 3. 确保机械臂线程也已结束
            arm_sweep_center_thread.join()

                # ==================== 2. 高潮：同步画圆 ====================
            # self.get_logger().info("Showtime: Starting the circle dance!")
            # msg = set_pose_target([self.x_init - 0.05, 0, self.z_dis], 0.0, [-180.0, 180.0], 1.0)
            # res = self.send_request(self.arm_kinematics_client, msg)
            # if res and res.pulse:
            #     set_servo_position(self.joints_pub, 1.0, ((24, 500), (23, 500), (22, res.pulse[3]), (21, res.pulse[2]), (20, res.pulse[1]), (19, res.pulse[0])))
            # # if interruptible_sleep(1): return
            # target_body_pose = kinematics_calculate.transform_euler(tuple(build_in_pose.DEFAULT_POSE), 
            #                                             (-30 , 0, 0),
            #                                             'xyz', (0,0,0), degrees=False)
                        
            # self.step_controller.set_pose_base(target_body_pose, 1.0)
            # if interruptible_sleep(1): return
            # # --- 动作参数 ---
            # body_radius_mm = 30
            # arm_radius_mm = 50
            # step_time = 0.04  # 稍微放慢一点，动作更清晰
            
            # # --- 内部辅助函数，用于执行一圈完整的画圆 ---
            # def perform_one_circle(reverse_direction=False):
            #     direction_str = "REVERSE" if reverse_direction else "FORWARD"
            #     self.get_logger().info(f"--- Performing {direction_str} circle ---")

            #     # 1. 生成路径点
            #     body_points = self.gen_circle(body_radius_mm, start_angle_deg=180, reverse=reverse_direction)
            #     arm_points = self.gen_circle(arm_radius_mm, start_angle_deg=180, reverse=reverse_direction)
                
            #     # 持续时间由路径点数量决定 (减1是因为我们先移动到第一个点)
            #     duration = (len(body_points) - 1) * step_time

            #     # 2. 【关键】平滑移动到路径的第一个点，避免“冲出去”
            #     self.get_logger().info("Moving smoothly to the starting point of the circle...")
            #     first_body_pt = body_points[0]
            #     first_arm_pt = arm_points[0]
                
            #     # 获取当前姿态作为变换基础
            #     base_pose = tuple(build_in_pose.DEFAULT_POSE)
                
            #     # 计算目标姿态
            #     target_body_pose = kinematics_calculate.transform_euler(base_pose, 
            #                                                             (first_body_pt[0] , first_body_pt[1], 0),
            #                                                             'xyz', (0,0,0), degrees=False)
                
            #     # 用1秒时间平滑移动身体和机械臂
                
            #     msg = set_pose_target([self.x_init + first_arm_pt[0]/1000.0, first_arm_pt[1]/1000.0, self.z_dis], 0.0, [-180.0, 180.0], 1.0)
            #     res = self.send_request(self.arm_kinematics_client, msg)
            #     if res and res.pulse:
            #         set_servo_position(self.joints_pub, 0.5, ((24, 500), (23, 500), (22, res.pulse[3]), (21, res.pulse[2]), (20, res.pulse[1]), (19, res.pulse[0])))
                
            #     self.step_controller.set_pose_base(target_body_pose, 0.5)

            #     # # 等待这个准备动作完成
            #     # if interruptible_sleep(0.5): return True # 返回True表示被中断

            #     # 3. 并行执行画圆路径
            #     self.get_logger().info("Starting parallel circle motion...")
            #     arm_thread = threading.Thread(target=self._perform_arm_path, args=(arm_points, duration))
            #     arm_thread.daemon = True
            #     arm_thread.start()

            #     # 主线程控制身体移动 (从第二个点开始)
            #     for x_mm, y_mm in body_points[1:]:
            #         if self._should_stop: break
            #         pose = kinematics_calculate.transform_euler(base_pose, (x_mm, y_mm , 0), 'xyz', (0,0,0), degrees=False)
            #         self.step_controller.set_pose_base(pose, step_time)
            #         if interruptible_sleep(step_time): break
                
            #     arm_thread.join() # 等待机械臂动作完成
            #     return self._should_stop # 返回是否被中断

            # # --- 执行舞蹈流程 ---
            # # 阶段一：正转一圈
            # if perform_one_circle(reverse_direction=False): return
            # if perform_one_circle(reverse_direction=False): return

            # # 阶段二：反转一圈
            # if perform_one_circle(reverse_direction=True): return
            # if perform_one_circle(reverse_direction=True): return

            # # ==================== 3. 结束动作 ====================
            # self.get_logger().info("Showtime: Taking a final bow!")
            # self.step_controller.set_build_in_pose('DEFAULT_POSE', 1.5)
            # # 恢复机械臂
            # msg = set_pose_target([self.x_init, 0.0, self.z_dis], 0.0, [-180.0, 180.0], 1.0)
            # res = self.send_request(self.arm_kinematics_client, msg)
            # if res and res.pulse:
            #     set_servo_position(self.joints_pub, 1.5, ((24, 500), (23, 500), (22, res.pulse[3]), (21, res.pulse[2]), (20, res.pulse[1]), (19, res.pulse[0])))
            # if interruptible_sleep(1.7): return

            
            # # # ==================== 新增部分：前进后退与机械臂协同 ====================
            # # self.get_logger().info("Showtime: Starting forward/backward motion with arm reach!")

            # # --- 动作参数 ---
            # # 使用 execute_traveling_and_wait 函数来控制身体移动
            # travel_period = 0.5  # 每走一步的时间
            # travel_steps = 4     # 一共走4步
            # travel_duration = travel_period * travel_steps # 身体移动的总时长 (2.0秒)

            # # --- 动作一：身体前进，同时机械臂向后收缩 ---
            # self.get_logger().info("--> Body FORWARD, Arm RETRACTING")

            # # 1. 在新线程中启动机械臂向后收缩的动作
            # #    从向前伸出0.05米(5cm)的位置，收回到向后0.05米的位置
            # arm_retract_thread = threading.Thread(target=self._perform_arm_reach, args=(0.0, -0.1, travel_duration))
            # arm_retract_thread.daemon = True
            # arm_retract_thread.start()

            # # 2. 主线程执行身体前进步态，并等待其完成
            # #    direction=0 表示正前方
            # if execute_traveling_and_wait(gait=1, stride=35, height=15, direction=math.radians(0), rotation=0, period=travel_period, steps=travel_steps): return

            # # 3. 确保机械臂线程也已结束
            # arm_retract_thread.join()

            # # --- 动作二：身体后退，同时机械臂向前伸出 ---
            # self.get_logger().info("--> Body BACKWARD, Arm EXTENDING")

            # # 1. 在新线程中启动机械臂向前伸出的动作
            # #    从向后0.05米的位置，伸展回向前0.05米的位置
            # arm_extend_thread = threading.Thread(target=self._perform_arm_reach, args=(-0.1, 0.0, travel_duration))
            # arm_extend_thread.daemon = True
            # arm_extend_thread.start()

            # # 2. 主线程执行身体后退步态
            # #    direction=180 表示正后方
            # if execute_traveling_and_wait(gait=1, stride=35, height=15, direction=math.radians(180), rotation=0, period=travel_period, steps=travel_steps): return

            # # 3. 确保机械臂线程也已结束
            # arm_extend_thread.join()

            # # ==================== 新增部分：平滑漂移与机械臂协同 ====================
            # self.get_logger().info("Showtime: Starting smooth forward/backward drift with arm reach!")

            # # --- 动作参数 ---
            # drift_distance = 10  # 身体向前/向后漂移的距离 (4厘米)
            # drift_duration = 1.0   # 完成一次漂移动作的总时长 (2.0秒)

            # # --- 动作一：身体向前平滑漂移，同时机械臂向后收缩 ---
            # self.get_logger().info("--> Body Drifting FORWARD, Arm RETRACTING")

            # # 1. 在新线程中启动机械臂向后收缩的动作
            # #    从中心位置向后移动10厘米
            # arm_retract_thread = threading.Thread(target=self._perform_arm_reach, args=(0.0, -0.1, drift_duration))
            # arm_retract_thread.daemon = True
            # arm_retract_thread.start()

            # # 2. 主线程执行身体向前平滑移动 (这是阻塞的)
            # #    transform_pose_euler 的 translate 参数是相对于当前姿态的增量
            # #    所以我们向前移动 drift_distance
            # # self.step_controller.transform_pose_euler(translate=(drift_distance, 0, 0), axis='xyz', euler=(0, 0, 0), duration=drift_duration)
            # self.step_controller.transform_pose_euler((80, 0, 0), 'xyz', (0, 0, 0), 1) # 平移， 单位为 mm

            # # 检查是否在身体移动过程中被中断
            # if self._should_stop: 
            #     arm_retract_thread.join() # 确保线程结束
            #     return

            # # 3. 确保机械臂线程也已结束
            # arm_retract_thread.join()

            # # --- 动作二：身体向后平滑漂移，同时机械臂向前伸出 ---
            # self.get_logger().info("--> Body Drifting BACKWARD, Arm EXTENDING")

            # # 1. 在新线程中启动机械臂向前伸出的动作
            # #    从后方10厘米的位置移动回中心
            # arm_extend_thread = threading.Thread(target=self._perform_arm_reach, args=(-0.1, 0.1, 2*drift_duration))
            # arm_extend_thread.daemon = True
            # arm_extend_thread.start()

            # # 2. 主线程执行身体向后平滑移动 (这也是阻塞的)
            # #    从当前的前倾位置，向后移动 drift_distance，回到原点
            # # self.step_controller.transform_pose_euler(translate=(-drift_distance, 0, 0), axis='xyz', euler=(0, 0, 0), duration=drift_duration)
            # self.step_controller.transform_pose_euler((-160, 0, 0), 'xyz', (0, 0, 0), 2) # 平移， 单位为 mm

            # # 检查是否在身体移动过程中被中断
            # if self._should_stop:
            #     arm_extend_thread.join() # 确保线程结束
            #     return

            # # 3. 确保机械臂线程也已结束
            # arm_extend_thread.join()
            
            # # 1. 在新线程中启动机械臂向后收缩的动作
            # #    从中心位置向后移动10厘米
            # arm_retract_thread = threading.Thread(target=self._perform_arm_reach, args=(0.1, 0.0, drift_duration))
            # arm_retract_thread.daemon = True
            # arm_retract_thread.start()

            # # 2. 主线程执行身体向前平滑移动 (这是阻塞的)
            # #    transform_pose_euler 的 translate 参数是相对于当前姿态的增量
            # #    所以我们向前移动 drift_distance
            # # self.step_controller.transform_pose_euler(translate=(drift_distance, 0, 0), axis='xyz', euler=(0, 0, 0), duration=drift_duration)
            # self.step_controller.transform_pose_euler((80, 0, 0), 'xyz', (0, 0, 0), 1) # 平移， 单位为 mm

            # # 检查是否在身体移动过程中被中断
            # if self._should_stop: 
            #     arm_retract_thread.join() # 确保线程结束
            #     return

            # # 3. 确保机械臂线程也已结束
            # arm_retract_thread.join()
 
            # # ==================== 新增部分：旋转升降与机械臂协同 ====================
            # self.get_logger().info("Showtime: Starting smooth forward/backward drift with arm reach!")

            # # --- 动作参数 ---
            # drift_distance = 10  # 身体向前/向后漂移的距离 (4厘米)
            # drift_duration = 1.0   # 完成一次漂移动作的总时长 (2.0秒)

            # # --- 动作一：身体向前平滑漂移，同时机械臂向后收缩 ---
            # self.get_logger().info("--> Body Drifting FORWARD, Arm RETRACTING")

            # # 1. 在新线程中启动机械臂向后收缩的动作
            # #    从中心位置向后移动10厘米
            # arm_thread_1 = threading.Thread(target=self._perform_arm_waving, args=(4, 19, 500))
            # arm_thread_1.daemon = True
            # arm_thread_1.start()

            # moves = [
            #     ((0, 0, 30), (0, 0, -20), 0.5),
            #     ((0, 0, -30), (0, 0, 20), 0.5),
            #     ((0, 0, 30), (0, 0, 20), 0.5),
            #     ((0, 0, -30), (0, 0, -20), 0.5),
                
            #     ((0, 0, 30), (0, 0, -20), 0.5),
            #     ((0, 0, -30), (0, 0, 20), 0.5),
            #     ((0, 0, 30), (0, 0, 20), 0.5),
            #     ((0, 0, -30), (0, 0, -20), 0.5),
            #     ]
            # for move in moves:
            #     # 解包参数
            #     translate, translation, duration = move
                
            #     # 执行动作并检查中断
            #     self.step_controller.transform_pose_euler(translate, 'xyz', translation, duration) # 平移， 单位为 mm
            #     if interruptible_sleep(duration): return

            # # 检查是否在身体移动过程中被中断
            # if self._should_stop: 
            #     arm_thread_1.join() # 确保线程结束
            #     return

            # # 3. 等待机械臂动作完成
            # arm_thread_1.join()

 
            # # ==================== 新增部分：旋转升降与机械臂协同 ====================
            # self.get_logger().info("Showtime: Starting smooth forward/backward drift with arm reach!")

            # # --- 动作参数 ---
            # drift_distance = 10  # 身体向前/向后漂移的距离 (4厘米)
            # drift_duration = 1.0   # 完成一次漂移动作的总时长 (2.0秒)

            # # --- 动作一：身体向前平滑漂移，同时机械臂向后收缩 ---
            # self.get_logger().info("--> Body Drifting FORWARD, Arm RETRACTING")

            # # 1. 在新线程中启动机械臂向后收缩的动作
            # #    从中心位置向后移动10厘米
            # arm_thread_1 = threading.Thread(target=self._perform_arm_waving_1, args=(1, 19, 650))
            # arm_thread_1.daemon = True
            # arm_thread_1.start()

            # moves_1 = [
            #     # 第一部分轨迹
            #     ((0, 10, 0), 0.5),
            #     ((-10, -10, 0), 0.5),
            #     ((10, -10, 0), 0.5),
            #     ((10, 10, 0), 0.5),
            #     ((0, 10, 0), 0.5),  
                
            #     # 第二部分轨迹
            #     ((10, -10, 0), 0.5),
            #     ((-10, -10, 0), 0.5),
            #     ((-10, 10, 0), 0.5),
            #     ((10, 10, 0), 0.5),
            #     ((0, -10, 0), 0.5),
            #     ]
            # for move in moves_1:
            #     # 解包参数
            #     translation, duration = move
                
            #     # 执行动作并检查中断
            #     self.step_controller.transform_pose_euler((0, 0, 0), 'xyz', translation, duration) # 平移， 单位为 mm
            #     if interruptible_sleep(duration): return

            # # 检查是否在身体移动过程中被中断
            # if self._should_stop: 
            #     arm_thread_1.join() # 确保线程结束
            #     return

            # # 3. 等待机械臂动作完成
            # arm_thread_1.join()

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