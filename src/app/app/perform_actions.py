#!/usr/bin/env python3
# encoding: utf-8
# Date:2022/10/20
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
    # --- 为舞蹈动作定义的常量数据序列 ---
    # 身体旋转摆动序列 (平移x,y,z, 欧拉角x,y,z, 持续时间)
    _BODY_TWIST_SEQUENCE = [
        ((0, 0, 30), (0, 0, -20), 0.5), ((0, 0, -30), (0, 0, 20), 0.5),
        ((0, 0, 30), (0, 0, 20), 0.5), ((0, 0, -30), (0, 0, -20), 0.5),
    ] * 2  # 重复两次

    # 身体菱形移动序列 (平移, 轴, 欧拉角, 持续时间)
    _DIAMOND_WALK_SEQUENCE = [
        ((0, 0, 0), 'xyz', (0, 10, 0), 0.5), ((-10, -10, 0), 0.5),
        ((10, -10, 0), 0.5), ((10, 10, 0), 0.5), ((0, 10, 0), 0.5),
        ((10, -10, 0), 0.5), ((-10, -10, 0), 0.5), ((-10, 10, 0), 0.5),
        ((10, 10, 0), 0.5), ((0, -10, 0), 0.5),
    ]

    def __init__(self):
        super().__init__('perform_actions')
        self.z_dis = 0.30
        self.x_init = transform.link3 + transform.tool_link
        self._action_lock = threading.Lock()
        self._is_running = False
        self._should_stop = False
        self._action_thread = None  # 持有当前动作线程的引用

        # ROS2 发布器、订阅与客户端
        self.joints_pub = self.create_publisher(ServosPosition, '/servo_controller', 1)
        self.create_subscription(RunActionSet, '~/actions', self.actions_callback, 1)
        self.arm_kinematics_client = self.create_client(SetRobotPose, '/arm_kinematics/set_pose_target')
        self.arm_kinematics_client.wait_for_service()

        # 运动控制器
        self.step_controller = step_controller.StepController()
        self.controller = ControllerClient()

    def send_request(self, client, msg):
        """异步发送服务请求并阻塞等待结果。"""
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def actions_callback(self, msg: RunActionSet):
        """根据收到的消息启动或停止动作。"""
        action_name = msg.action_path
        if action_name == 'stop':
            self.get_logger().info("收到停止指令。")
            self.stop_current_action()
        else:
            self.get_logger().info(f"收到开始指令: {action_name}")
            self.start_action(action_name)

    def start_action(self, action_name):
        """启动一个新的后台动作线程。"""
        with self._action_lock:
            if self._is_running:
                self.get_logger().warn("已有动作正在运行，请先停止。")
                return
        action_map = {
            'twist': self.twist, 'turn_round': self.turn_round,
            'dance_2': self.dance_routine, 'dance_1': self.dance,
        }
        target_func = action_map.get(action_name)
        if not target_func:
            self.get_logger().error(f"未知的动作名称: {action_name}")
            return
        self._action_thread = threading.Thread(target=target_func)
        self._action_thread.daemon = True
        self._action_thread.start()

    def stop_current_action(self):
        """设置停止标志并等待当前动作线程结束。"""
        self._should_stop = True
        self.get_logger().info("已设置停止标志，等待动作线程终止...")
        if self._action_thread and self._action_thread.is_alive():
            self._action_thread.join(timeout=1.0)
        self.get_logger().info("动作已停止。")

    def _interruptible_sleep(self, duration):
        """可被中断的睡眠函数。"""
        start_time = time.time()
        while time.time() - start_time < duration:
            if self._should_stop: return True
            time.sleep(0.02)
        return False

    def _set_running_state(self, is_running):
        """统一管理动作的运行状态和停止标志。"""
        with self._action_lock:
            self._is_running = is_running
        if not is_running:
            self._should_stop = False

    def _execute_choreography(self, body_moves, parallel_arm_action=None):
        """执行一段身体动作序列，并可选择性地并行执行一个机械臂动作。"""
        arm_thread = None
        if parallel_arm_action:
            target, args = parallel_arm_action
            arm_thread = threading.Thread(target=target, args=args)
            arm_thread.daemon = True
            arm_thread.start()
        for move_params in body_moves:
            if self._should_stop: break
            if len(move_params) == 3:
                translate, euler, duration = move_params
                self.step_controller.transform_pose_euler(translate, 'xyz', euler, duration)
            elif len(move_params) == 4:
                translate, axis, euler, duration = move_params
                self.step_controller.transform_pose_euler(translate, axis, euler, duration)
            else: # 兼容菱形舞步中只有平移和时长的元组
                translation, duration = move_params
                self.step_controller.transform_pose_euler((0,0,0), 'xyz', translation, duration)
            if self._interruptible_sleep(duration): break
        if arm_thread:
            if self._should_stop: self.get_logger().info("主流程中断，等待并行机械臂动作结束...")
            arm_thread.join()
        return self._should_stop

    ### --- 主要动作函数 --- ###

    def twist(self):
        """机体扭动动作。"""
        self._set_running_state(True)
        self.get_logger().info("动作 [Twist] 开始。")
        try:
            # ... (此函数逻辑保持不变) ...
            duration = 0.03
            org_pose = tuple(build_in_pose.DEFAULT_POSE_M)
            if self._interruptible_sleep(0.8): return
            for j in range(7, 16, 2):
                if self._should_stop: return
                i = 90; j = min(15, j)
                while i <= 360 + 90:
                    if self._should_stop: return
                    t = 0.5 if i == 90 and j == 7 else duration
                    i += 4 + j * 0.30
                    x = math.sin(math.radians(i)) * (0.018 * (j + ((i - 90) / 360) * 2))
                    y = math.cos(math.radians(i)) * (0.018 * (j + ((i - 90) / 360) * 2))
                    pose = kinematics_calculate.transform_euler(org_pose, (0, 0, 0), 'xy', (x, y), degrees=False)
                    self.step_controller.set_pose_base(pose, t)
                    if self._interruptible_sleep(t): return
            for j in range(16, 7, -2):
                if self._should_stop: return
                i = 360 + 90
                while i >= 90:
                    if self._should_stop: return
                    t = 0.5 if i == 90 and j == 7 else duration
                    i += -(4 + j * 0.30)
                    x = math.sin(math.radians(i)) * (0.018 * (j + (1 - (i - 90) / 360) * -2))
                    y = math.cos(math.radians(i)) * (0.018 * (j + (1 - (i - 90) / 360) * -2))
                    pose = kinematics_calculate.transform_euler(org_pose, (0, 0, 0), 'xy', (x, y), degrees=False)
                    self.step_controller.set_pose_base(pose, t)
                    if self._interruptible_sleep(t): return
            if self._should_stop: return
            self.step_controller.set_build_in_pose('DEFAULT_POSE_M', 1)
            if self._interruptible_sleep(1): return
            self.get_logger().info("动作 [Twist] 成功结束。")
        finally:
            self._set_running_state(False)
            self.get_logger().info("动作状态已重置。")

    def turn_round(self):
        """原地扭身动作。"""
        self._set_running_state(True)
        self.get_logger().info("动作 [Turn Round] 开始。")
        try:
            # ... (此函数逻辑保持不变) ...
            self.step_controller.set_build_in_pose('DEFAULT_POSE_M', 0.8)
            if self._interruptible_sleep(0.8): return
            org_pose = tuple(build_in_pose.DEFAULT_POSE_M)
            pose_right = kinematics_calculate.transform_euler(org_pose, (0, 0, 30), 'xyz', (0, 0, 0.8), degrees=False)
            pose_left = kinematics_calculate.transform_euler(org_pose, (0, 0, -30), 'xyz', (0, 0, -0.8), degrees=False)
            self.step_controller.set_pose_base(pose_right, 0.3)
            if self._interruptible_sleep(0.3): return
            for _ in range(7):
                if self._should_stop: return
                self.step_controller.set_pose_base(pose_left, 0.6)
                if self._interruptible_sleep(0.6): return
                if self._should_stop: return
                self.step_controller.set_pose_base(pose_right, 0.6)
                if self._interruptible_sleep(0.6): return
            self.step_controller.set_pose_base(build_in_pose.DEFAULT_POSE_M, 1)
            self.get_logger().info("动作 [Turn Round] 成功结束。")
        finally:
            self._set_running_state(False)
            self.get_logger().info("动作状态已重置。")

    def dance_routine(self):
        """一段富有表演性的舞蹈程序（可中断）。"""
        self._set_running_state(True)
        self.get_logger().info("动作 [Dance Routine] 开始。")
        try:
            # ... (此函数逻辑保持不变) ...
            def execute_traveling_and_wait(gait, stride, height, direction, rotation, period, steps):
                self.step_controller.set_step_mode(gait, stride, height, direction, rotation, period, steps)
                self.step_controller.stopped.clear()
                while not self.step_controller.stopped.is_set():
                    if self._should_stop: self.step_controller.stop_running(); return True
                    time.sleep(0.1)
                return False
            self.step_controller.set_build_in_pose('DEFAULT_POSE', 1.0)
            if self._interruptible_sleep(1.2): return
            self.get_logger().info("舞蹈: 点头..."); self.step_controller.transform_pose_euler((0,0,0), 'xyz', (15,0,0), 0.8);
            if self._interruptible_sleep(1.0): return; self.step_controller.transform_pose_euler((0,0,0), 'xyz', (-15,0,0), 0.8);
            if self._interruptible_sleep(1.0): return
            self.get_logger().info("舞蹈: 扭腰...");
            if self.simple_wave(): return
            self.get_logger().info("舞蹈: 菱形舞步!"); steps, step_time = 4, 0.5
            if execute_traveling_and_wait(1, 35, 15, math.radians(45), 0, step_time, steps): return
            if execute_traveling_and_wait(1, 35, 15, math.radians(-45), 0, step_time, steps): return
            if execute_traveling_and_wait(1, 35, 15, math.radians(-135), 0, step_time, steps): return
            if execute_traveling_and_wait(1, 35, 15, math.radians(135), 0, step_time, steps): return
            self.get_logger().info("舞蹈: 最终扭腰!");
            if self.simple_wave(): return
            self.get_logger().info("舞蹈: 谢幕!"); self.step_controller.transform_pose_euler((0,0,-35), 'xyz', (0,0.5,0), 1.5);
            if self._interruptible_sleep(1.7): return
            self.step_controller.set_build_in_pose('DEFAULT_POSE', 2.0)
            if self._interruptible_sleep(2.2): return
            self.get_logger().info("动作 [Dance Routine] 成功结束。")
        finally:
            self.step_controller.stop_running(); self.step_controller.set_build_in_pose('DEFAULT_POSE', 1.0)
            self._set_running_state(False); self.get_logger().info("动作状态已重置。")

    def simple_wave(self):
        """一个简化的、可中断的身体摇摆动作。"""
        for _ in range(2):
            for angle in [-15, 15, 0]:
                if self._should_stop: return True
                self.step_controller.transform_pose_euler((0,0,0), 'xyz', (math.radians(angle),0,0), 0.4)
                if self._interruptible_sleep(0.5): return
        return False

    ### --- 机械臂辅助动作函数 (保留所有) --- ###
    def _perform_arm_waving_1(self, repetitions: int, arm_joint_id: int, arm_target_pulse: int):
        self.get_logger().info("机械臂动作 (数据列表版) 开始。")
        try:
            moves = [(0.03, 500), (-0.03, 400), (-0.03, 500), (0.03, 600), (0.03, 500),
                     (-0.03, 600), (-0.03, 500), (0.03, 400), (0.0, 500)]
            for _ in range(repetitions):
                for dis, pulse in moves:
                    if self._should_stop: break
                    msg = set_pose_target([self.x_init, 0.0, self.z_dis + dis], 0.0, [-180.0, 180.0], 1.0)
                    res = self.send_request(self.arm_kinematics_client, msg)
                    if res and res.pulse:
                        set_servo_position(self.joints_pub, 0.5, ((24, 500), (23, 500), (22, res.pulse[3]), (21, res.pulse[2]), (20, res.pulse[1]), (19, pulse)))
                    if self._interruptible_sleep(0.5): return
                if self._should_stop: break
        finally:
            self.get_logger().info("机械臂动作 (数据列表版) 结束。")

    def _perform_arm_waving(self, repetitions: int, arm_joint_id: int, arm_target_pulse: int):
        self.get_logger().info("机械臂挥舞线程开始。")
        try:
            for i in range(repetitions):
                if self._should_stop: break
                msg = set_pose_target([self.x_init, 0.0, self.z_dis + 0.03], 0.0, [-180.0, 180.0], 1.0)
                res = self.send_request(self.arm_kinematics_client, msg)
                if res and res.pulse:
                    set_servo_position(self.joints_pub, 0.5, ((24, 500), (23, 500), (22, res.pulse[3]), (21, res.pulse[2]), (20, res.pulse[1]), (arm_joint_id, arm_target_pulse)))
                if self._interruptible_sleep(0.5): return
                if self._should_stop: break
                msg = set_pose_target([self.x_init, 0.0, self.z_dis - 0.03], 0.0, [-180.0, 180.0], 1.0)
                res = self.send_request(self.arm_kinematics_client, msg)
                if res and res.pulse:
                    set_servo_position(self.joints_pub, 0.5, ((24, 500), (23, 500), (22, res.pulse[3]), (21, res.pulse[2]), (20, res.pulse[1]), (arm_joint_id, arm_target_pulse)))
                if self._interruptible_sleep(0.5): return
        finally:
            self.get_logger().info("机械臂挥舞线程结束。")

    def _perform_arm_sweep(self, start_y, end_y, duration_secs):
        self.get_logger().info(f"机械臂扫动: y 从 {start_y} 到 {end_y}。")
        try:
            positions = np.linspace(start_y, end_y, num=20)
            step_duration = duration_secs / len(positions)
            for y_pos in positions:
                if self._should_stop: return
                msg = set_pose_target([self.x_init, y_pos, self.z_dis], 0.0, [-180.0, 180.0], 1.0)
                res = self.send_request(self.arm_kinematics_client, msg)
                if res and res.pulse:
                    set_servo_position(self.joints_pub, step_duration, ((24, 500), (23, 500), (22, res.pulse[3]), (21, res.pulse[2]), (20, res.pulse[1]), (19, res.pulse[0])))
                if self._interruptible_sleep(step_duration): return
        finally:
            self.get_logger().info("机械臂扫动结束。")

    def gen_circle(self, r, start_angle_deg=180, reverse=False):
        points = []
        angles = np.arange(0, 360, 5)
        start_index = int(start_angle_deg / 5) % len(angles)
        ordered_angles_deg = np.roll(angles, -start_index)
        if reverse:
            ordered_angles_deg = np.concatenate(([ordered_angles_deg[0]], np.flip(ordered_angles_deg[1:])))
        for angle_deg in ordered_angles_deg:
            rad = math.radians(angle_deg)
            x = r * math.cos(rad); y = r * math.sin(rad)
            points.append((x, y))
        points.append(points[0])
        return points

    def wait_for_action(self, duration):
        end_time = time.time() + duration
        while time.time() < end_time:
            if self._should_stop: return True
        return False

    def _perform_arm_path(self, points_mm, duration_secs):
        self.get_logger().info(f"机械臂路径运动开始，共 {len(points_mm)} 个点。")
        try:
            if len(points_mm) <= 1: return
            path_to_traverse = points_mm[1:]
            step_duration = duration_secs / len(path_to_traverse)
            for x_mm, y_mm in path_to_traverse:
                if self._should_stop: return
                target_x = self.x_init + (x_mm / 1000.0); target_y = y_mm / 1000.0
                msg = set_pose_target([target_x, target_y, self.z_dis], 0.0, [-180.0, 180.0], 1.0)
                res = self.send_request(self.arm_kinematics_client, msg)
                if res and res.pulse:
                    set_servo_position(self.joints_pub, step_duration, ((24, 500), (23, 500), (22, res.pulse[3]), (21, res.pulse[2]), (20, res.pulse[1]), (19, res.pulse[0])))
                if self.wait_for_action(step_duration): return
        finally:
            self.get_logger().info("机械臂路径运动结束。")

    def _perform_arm_reach(self, start_x_offset, end_x_offset, duration_secs):
        self.get_logger().info(f"机械臂伸缩: x_offset 从 {start_x_offset} 到 {end_x_offset}。")
        try:
            positions = np.linspace(start_x_offset, end_x_offset, num=20)
            if len(positions) == 0: return
            step_duration = duration_secs / len(positions)
            for x_offset in positions:
                if self._should_stop: return
                target_x = self.x_init - 0.02 + x_offset
                # target_x = x_offset
                msg = set_pose_target([target_x, 0.0, self.z_dis], 0.0, [-180.0, 180.0], 1.0)
                res = self.send_request(self.arm_kinematics_client, msg)
                if res and res.pulse:
                    set_servo_position(self.joints_pub, step_duration, ((24, 500), (23, 500), (22, res.pulse[3]), (21, res.pulse[2]), (20, res.pulse[1]), (19, res.pulse[0])))
                if self._interruptible_sleep(step_duration): return
        finally:
            self.get_logger().info("机械臂伸缩结束。")

    def _perform_arm_nod(self, repetitions, duration_per_nod):
        self.get_logger().info(f"机械臂点头动作开始，重复 {repetitions} 次。")
        original_z = self.z_dis; nod_down_z = original_z - 0.04
        try:
            for _ in range(repetitions):
                if self._should_stop: return
                msg = set_pose_target([self.x_init, 0.0, nod_down_z], 0.0, [-180.0, 180.0], 1.0)
                res = self.send_request(self.arm_kinematics_client, msg)
                if res and res.pulse:
                    set_servo_position(self.joints_pub, duration_per_nod / 2, ((24, 500), (23, 500), (22, res.pulse[3]), (21, res.pulse[2]), (20, res.pulse[1]), (19, res.pulse[0])))
                if self._interruptible_sleep(duration_per_nod / 2): return
                if self._should_stop: return
                msg = set_pose_target([self.x_init, 0.0, original_z], 0.0, [-180.0, 180.0], 1.0)
                res = self.send_request(self.arm_kinematics_client, msg)
                if res and res.pulse:
                    set_servo_position(self.joints_pub, duration_per_nod / 2, ((24, 500), (23, 500), (22, res.pulse[3]), (21, res.pulse[2]), (20, res.pulse[1]), (19, res.pulse[0])))
                if self._interruptible_sleep(duration_per_nod / 2): return
        finally:
            msg = set_pose_target([self.x_init, 0.0, original_z], 0.0, [-180.0, 180.0], 1.0)
            res = self.send_request(self.arm_kinematics_client, msg)
            if res and res.pulse:
                set_servo_position(self.joints_pub, 0.3, ((24, 500), (23, 500), (22, res.pulse[3]), (21, res.pulse[2]), (20, res.pulse[1]), (19, res.pulse[0])))
            self.get_logger().info("机械臂点头动作结束。")

    ### --- Dance 组合动作辅助函数 --- ###
    def _execute_traveling_and_wait(self, gait, stride, height, direction, rotation, period, steps):
        """执行一个阻塞的、可中断的行进步态。"""
        self.step_controller.set_step_mode(gait, stride, height, direction, rotation, period, steps)
        self.step_controller.stopped.clear()
        while not self.step_controller.stopped.is_set():
            if self._should_stop:
                self.step_controller.stop_running()
                return True
            time.sleep(0.1)
        return False

    def _step_and_wave_arm(self, turn_angle, arm_pulse, wave_reps):
        """组合动作：转身+踏步+挥臂。"""
        self.get_logger().info(f"组合动作: 转身 {turn_angle}度 并挥舞机械臂。")
        self.step_controller.transform_pose_euler((0,0,0), 'xyz', (0,0,turn_angle), 0.5)
        set_servo_position(self.joints_pub, 0.5, ((19, arm_pulse),))
        if self._interruptible_sleep(0.5): return True
        self.step_controller.set_step_mode(2, 0, 15, 0, 0, 0.5, 0)
        self.step_controller.stopped.clear()
        self._perform_arm_waving(wave_reps, 19, arm_pulse)
        self.step_controller.stop_running()
        while not self.step_controller.stopped.is_set():
            if self._should_stop: return True
            time.sleep(0.1)
        return self._should_stop

    def _translate_and_sweep_arm(self, direction_deg, steps, period, arm_start_y, arm_end_y):
        """组合动作：身体平移+机械臂扫动。"""
        self.get_logger().info(f"组合动作: 向 {direction_deg}度 平移并扫动机械臂。")
        travel_duration = period * steps
        arm_thread = threading.Thread(target=self._perform_arm_sweep, args=(arm_start_y, arm_end_y, travel_duration))
        arm_thread.daemon = True
        arm_thread.start()
        if self._execute_traveling_and_wait(2, 25, 15, math.radians(direction_deg), 0, period, steps):
            arm_thread.join()
            return True
        arm_thread.join()
        return self._should_stop

    def _walk_and_reach_arm(self, direction_deg, arm_start_offset, arm_end_offset):
        """组合动作：身体前进/后退+机械臂伸缩。"""
        self.get_logger().info(f"组合动作: 向 {direction_deg}度 行走并伸缩机械臂。")
        travel_period, travel_steps = 0.5, 4
        travel_duration = travel_period * travel_steps
        arm_thread = threading.Thread(target=self._perform_arm_reach, args=(arm_start_offset, arm_end_offset, travel_duration))
        arm_thread.daemon = True
        arm_thread.start()
        if self._execute_traveling_and_wait(1, 35, 15, math.radians(direction_deg), 0, travel_period, travel_steps):
            arm_thread.join()
            return True
        arm_thread.join()
        return self._should_stop

    def _drift_and_reach_arm(self, drift_vector, arm_start_offset, arm_end_offset, duration):
        """组合动作：身体平滑漂移+机械臂伸缩。"""
        self.get_logger().info(f"组合动作: 身体漂移 {drift_vector} 并伸缩机械臂。")
        arm_thread = threading.Thread(target=self._perform_arm_reach, args=(arm_start_offset, arm_end_offset, duration))
        arm_thread.daemon = True
        arm_thread.start()
        self.step_controller.transform_pose_euler(drift_vector, 'xyz', (0,0,0), duration)
        if self._should_stop:
            arm_thread.join()
            return True
        arm_thread.join()
        return self._should_stop

    ### --- 最终的 Dance 函数 --- ###
    def dance(self):
        """一个富有表现力的、模块化的机器人舞蹈表演（可中断）。"""
        self._set_running_state(True)
        self.get_logger().info("表演时刻：舞蹈开始！")
        try:
            # ==================== 1. 开场：准备与热身 ====================
            self.get_logger().info(">>> 阶段 1: 开场")
            self.step_controller.set_build_in_pose('DEFAULT_POSE', 1.5)
            msg = set_pose_target([self.x_init, 0.0, self.z_dis], 0.0, [-180.0, 180.0], 1.0)
            res = self.send_request(self.arm_kinematics_client, msg)
            if res.pulse:
                set_servo_position(self.joints_pub, 1.0, ((24, 500), (23, 500), (22, res.pulse[3]), (21, res.pulse[2]), (20, res.pulse[1]), (19, res.pulse[0])))
            if self._interruptible_sleep(1.0): return
            if self._execute_traveling_and_wait(2, 0, 15, 0, 0, 0.5, 2): return
            if self._step_and_wave_arm(turn_angle=15, arm_pulse=750, wave_reps=2): return
            if self._step_and_wave_arm(turn_angle=-30, arm_pulse=350, wave_reps=2): return
            set_servo_position(self.joints_pub, 0.5, ((19, 500),)); self.step_controller.set_build_in_pose('DEFAULT_POSE', 0.5)
            if self._interruptible_sleep(0.5): return

            # ==================== 2. 平移动作与机械臂协同 ====================
            self.get_logger().info(">>> 阶段 2: 平移与扫臂")
            if self._translate_and_sweep_arm(90, 8, 0.5, 0.0, 0.15): return
            if self._translate_and_sweep_arm(270, 8, 0.5, 0.15, -0.15): return
            if self._translate_and_sweep_arm(90, 4, 0.5, -0.15, 0.0): return
            
            target_body_pose = kinematics_calculate.transform_euler(tuple(build_in_pose.DEFAULT_POSE),  (-30 , 0, 0), 'xyz', (0,0,0), degrees=False)
            msg = set_pose_target([self.x_init - 0.05, 0, self.z_dis], 0.0, [-180.0, 180.0], 1.0)
            res = self.send_request(self.arm_kinematics_client, msg)
            if res and res.pulse: set_servo_position(self.joints_pub, 0.5, ((24, 500), (23, 500), (22, res.pulse[3]), (21, res.pulse[2]), (20, res.pulse[1]), (19, res.pulse[0])))
            if self._interruptible_sleep(1): return True
            self.step_controller.set_pose_base(target_body_pose, 0.5)
            if self._interruptible_sleep(0.5): return True
            # ==================== 3. 高潮：同步画圆 ====================
            def perform_one_circle(reverse=False):
                body_pts = self.gen_circle(30, 180, reverse); arm_pts = self.gen_circle(50, 180, reverse)
                duration = (len(body_pts) - 1) * 0.04
                # target_body_pose = kinematics_calculate.transform_euler(tuple(build_in_pose.DEFAULT_POSE),  (-30 , 0, 0), 'xyz', (0,0,0), degrees=False)
                # msg = set_pose_target([self.x_init - 0.05, 0, self.z_dis], 0.0, [-180.0, 180.0], 1.0)
                # res = self.send_request(self.arm_kinematics_client, msg)
                # if res and res.pulse: set_servo_position(self.joints_pub, 0.5, ((24, 500), (23, 500), (22, res.pulse[3]), (21, res.pulse[2]), (20, res.pulse[1]), (19, res.pulse[0])))
                # if self._interruptible_sleep(1): return True
                # self.step_controller.set_pose_base(target_body_pose, 0.5)
                # if self._interruptible_sleep(0.5): return True
                arm_thread = threading.Thread(target=self._perform_arm_path, args=(arm_pts, duration)); arm_thread.daemon = True; arm_thread.start()
                for x, y in body_pts[1:]:
                    if self._should_stop: break
                    pose = kinematics_calculate.transform_euler(tuple(build_in_pose.DEFAULT_POSE), (x, y, 0), 'xyz', (0,0,0), degrees=False)
                    self.step_controller.set_pose_base(pose, 0.04)
                    if self._interruptible_sleep(0.04): break
                arm_thread.join()
                return self._should_stop
            if perform_one_circle(reverse=False): return
            # if perform_one_circle(reverse=False): return
            # if perform_one_circle(reverse=True): return
            if perform_one_circle(reverse=True): return
            
            target_body_pose = kinematics_calculate.transform_euler(tuple(build_in_pose.DEFAULT_POSE),  (0 , 0, 0), 'xyz', (0,0,0), degrees=False)
            # msg = set_pose_target([self.x_init, 0, self.z_dis], 0.0, [-180.0, 180.0], 1.0)
            # res = self.send_request(self.arm_kinematics_client, msg)
            # if res and res.pulse: set_servo_position(self.joints_pub, 0.5, ((24, 500), (23, 500), (22, res.pulse[3]), (21, res.pulse[2]), (20, res.pulse[1]), (19, res.pulse[0])))
            # self.step_controller.set_pose_base(target_body_pose, 0.5)
            msg = set_pose_target([self.x_init - 0.03 , 0, self.z_dis], 0.0, [-180.0, 180.0], 1.0)
            res = self.send_request(self.arm_kinematics_client, msg)
            if res and res.pulse: set_servo_position(self.joints_pub, 0.5, ((24, 500), (23, 500), (22, res.pulse[3]), (21, res.pulse[2]), (20, res.pulse[1]), (19, res.pulse[0])))
            if self._interruptible_sleep(1): return True
            self.step_controller.set_pose_base(target_body_pose, 0.5)
            if self._interruptible_sleep(0.5): return True


            # # ==================== 4. 前后步态与机械臂协同 ====================
            # self.get_logger().info(">>> 阶段 4: 前后步态")
            if self._walk_and_reach_arm(0, 0.0, -0.1): return
            if self._walk_and_reach_arm(180, -0.1, 0.0): return

            # ==================== 5. 平滑漂移与机械臂协同 ====================
            self.get_logger().info(">>> 阶段 5: 平滑漂移")
            if self._drift_and_reach_arm((80, 0, 0), 0.0, -0.1, 1.0): return
            if self._drift_and_reach_arm((-160, 0, 0), -0.1, 0.1, 2.0): return
            if self._drift_and_reach_arm((80, 0, 0), 0.1, 0.0, 1.0): return
            
            # ==================== 6. 旋转升降与机械臂协同 ====================
            self.get_logger().info(">>> 阶段 6: 旋转升降")
            if self._execute_choreography(self._BODY_TWIST_SEQUENCE, parallel_arm_action=(self._perform_arm_waving, (4, 19, 500))): return

            # ==================== 7. 菱形舞步与机械臂协同 ====================
            self.get_logger().info(">>> 阶段 7: 菱形舞步")
            if self._execute_choreography(self._DIAMOND_WALK_SEQUENCE, parallel_arm_action=(self._perform_arm_waving_1, (1, 19, 650))): return

            # ==================== 8. 结束动作 ====================
            self.get_logger().info(">>> 阶段 8: 谢幕")
            self.step_controller.set_build_in_pose('DEFAULT_POSE', 1.5)
            msg = set_pose_target([self.x_init, 0.0, self.z_dis], 0.0, [-180.0, 180.0], 1.0)
            res = self.send_request(self.arm_kinematics_client, msg)
            if res and res.pulse:
                set_servo_position(self.joints_pub, 1.5, ((24, 500), (23, 500), (22, res.pulse[3]), (21, res.pulse[2]), (20, res.pulse[1]), (19, res.pulse[0])))
            if self._interruptible_sleep(1.7): return

        finally:
            self.get_logger().info("确保所有动作都已停止。")
            self.step_controller.stop_running()
            self.step_controller.set_build_in_pose('DEFAULT_POSE', 1.0)
            self._set_running_state(False)
            self.get_logger().info("舞蹈结束，状态已重置。")


def main():
    rclpy.init()
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
