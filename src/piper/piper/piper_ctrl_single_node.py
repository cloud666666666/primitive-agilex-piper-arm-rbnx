#!/usr/bin/env python3
# -*-coding:utf8-*-
# This file controls a single robotic arm node and handles the movement of the robotic arm with a gripper.
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import time
import threading
import argparse
import math
from piper_sdk import *
from piper_sdk import C_PiperInterface_V2
from piper_msgs.msg import PiperStatusMsg
from piper_msgs.srv import Enable
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R  # For Euler angle to quaternion conversion
from numpy import clip


class PiperRosNode(Node):
    """ROS2 node for the robotic arm"""

    JOINT_CTRL_LIMITS = {
        'joint1': (-150 * 1000, 150 * 1000),
        'joint2': (0, 180 * 1000),
        'joint3': (-170 * 1000, 0),
        'joint4': (-100 * 1000, 100 * 1000),
        'joint5': (-70 * 1000, 70 * 1000),
        'joint6': (-120 * 1000, 120 * 1000),
    }

    def __init__(self) -> None:
        super().__init__('piper_ctrl_single_node')
        # ROS parameters
        self.declare_parameter('can_port', 'can0')
        self.declare_parameter('auto_enable', False)
        self.declare_parameter('gripper_exist', True)
        self.declare_parameter('gripper_val_mutiple', 1)
        self.declare_parameter('gripper_stroke_m', 0.10)

        self.can_port = self.get_parameter('can_port').get_parameter_value().string_value
        self.auto_enable = self.get_parameter('auto_enable').get_parameter_value().bool_value
        self.gripper_exist = self.get_parameter('gripper_exist').get_parameter_value().bool_value
        self.gripper_val_mutiple = self.get_parameter('gripper_val_mutiple').get_parameter_value().integer_value
        self.gripper_val_mutiple = max(0, min(self.gripper_val_mutiple, 10))
        # 夹爪机械行程（米）。这台 Piper 夹爪上位机里显示是 100 mm；
        # SDK 自带的钳位只给到 70 mm，必须关掉（见下面 C_PiperInterface_V2 的
        # start_sdk_gripper_limit），再用这个值做软件限位。
        self.gripper_stroke_m = self.get_parameter('gripper_stroke_m').get_parameter_value().double_value
        if not 0.0 < self.gripper_stroke_m <= 0.10:
            self.get_logger().warning(
                f"gripper_stroke_m={self.gripper_stroke_m} 不合理，夹到 [0.001, 0.10]")
            self.gripper_stroke_m = max(0.001, min(self.gripper_stroke_m, 0.10))

        self.get_logger().info(f"can_port is {self.can_port}")
        self.get_logger().info(f"auto_enable is {self.auto_enable}")
        self.get_logger().info(f"gripper_exist is {self.gripper_exist}")
        self.get_logger().info(f"gripper_val_mutiple is {self.gripper_val_mutiple}")
        self.get_logger().info(
            f"gripper_stroke_m is {self.gripper_stroke_m}")
        # Publishers
        self.joint_pub = self.create_publisher(JointState, 'joint_states_single', 1)
        self.joint_ctrl_pub = self.create_publisher(JointState, 'joint_ctrl', 1)
        self.arm_status_pub = self.create_publisher(PiperStatusMsg, 'arm_status', 1)
        self.end_pose_pub = self.create_publisher(Pose, 'end_pose', 1)
        # Service
        self.motor_srv = self.create_service(Enable, 'enable_srv', self.handle_enable_service)
        # Joint
        self.joint_states = JointState()
        self.joint_states.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
        self.joint_states.position = [0.0] * 7
        self.joint_states.velocity = [0.0] * 7
        self.joint_states.effort = [0.0] * 7
        # Joint ctrl
        self.joint_ctrl = JointState()
        self.joint_ctrl.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
        self.joint_ctrl.position = [0.0] * 7
        self.joint_ctrl.velocity = [0.0] * 7
        self.joint_ctrl.effort = [0.0] * 7
        # Enable flag
        self.__enable_flag = False
        self._last_joint_clip_log_time = 0.0
        self._last_gripper_clip_log_time = 0.0
        # Create piper class and open CAN interface.
        # start_sdk_gripper_limit=False：SDK 默认会按 gripper_range=[0, 0.07]
        # （70 mm）把夹爪命令钳住，而这台夹爪的机械行程是 100 mm，钳住之后
        # 上层永远开不到满开口，比 70 mm 大的物体就"夹不住"（实测卡在 69.6 mm）。
        # sdk backend（piper_arm_sdk/main.py）同样传 False，两个后端保持一致；
        # 上限改由本节点的 gripper_stroke_m 做软件限位。
        self.piper = C_PiperInterface_V2(
            can_name=self.can_port, start_sdk_gripper_limit=False)
        self.piper.ConnectPort()
        self.get_logger().info(
            f"gripper SDK limit disabled; software stroke limit "
            f"{self.gripper_stroke_m * 1000:.1f} mm")

        # Start subscription thread
        # `pos_command` (geometry_msgs/Pose) — Cartesian end-effector target.
        # Gripper is NOT part of this contract; command the gripper as a
        # named joint on `joint_command` instead.
        self.create_subscription(Pose, 'pos_command', self.pos_callback, 1)
        # `joint_command` (sensor_msgs/JointState) — named joint target,
        # optionally including a `gripper` entry.
        self.create_subscription(JointState, 'joint_command', self.joint_callback, 1)
        self.create_subscription(Bool, 'enable_flag', self.enable_callback, 1)

        self.publisher_thread = threading.Thread(target=self.publish_thread)
        self.publisher_thread.start()

    def GetEnableFlag(self):
        return self.__enable_flag

    def _clip_joint_positions_for_sdk(self, joint_positions):
        clipped_positions = {}
        clipped_items = []
        for joint_name, (lower, upper) in self.JOINT_CTRL_LIMITS.items():
            value = int(joint_positions.get(joint_name, 0))
            clipped = max(lower, min(value, upper))
            clipped_positions[joint_name] = clipped
            if clipped != value:
                clipped_items.append(f"{joint_name}:{value}->{clipped}")

        if clipped_items:
            now = time.time()
            if now - self._last_joint_clip_log_time > 1.0:
                self.get_logger().warning(
                    "Clipped joint command to Piper SDK limits: "
                    + ", ".join(clipped_items)
                )
                self._last_joint_clip_log_time = now
        return clipped_positions

    def _clip_gripper_for_sdk(self, joint_6: int) -> int:
        """夹爪软件限位。

        SDK 侧的钳位被关掉之后由这里兜底：上层如果按更大的开口下发
        （grasp skill 的 `opening_mm` 常写成 102 mm），不能把超过机械行程
        的命令原样发给固件。
        """
        limit = int(round(self.gripper_stroke_m * 1000 * 1000))
        if joint_6 > limit:
            now = time.time()
            if now - self._last_gripper_clip_log_time > 1.0:
                self.get_logger().warning(
                    f"Clipped gripper command {joint_6 / 1000:.1f} mm "
                    f"-> {limit / 1000:.1f} mm (gripper_stroke_m)")
                self._last_gripper_clip_log_time = now
            return limit
        return max(0, joint_6)

    def publish_thread(self):
        """Publish messages from the robotic arm
        """
        rate = self.create_rate(10)  # 10 Hz
        enable_flag = False
        # Set timeout (seconds)
        timeout = 5
        # Record the time before entering the loop
        start_time = time.time()
        elapsed_time_flag = False
        while rclpy.ok():
            if(self.auto_enable):
                while not (enable_flag):
                    elapsed_time = time.time() - start_time
                    self.get_logger().info("--------------------")
                    enable_flag = self.piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status
                    self.get_logger().info(f"Enable status:{enable_flag}")
                    self.piper.EnableArm(7)
                    self.piper.GripperCtrl(0, 1000, 0x01, 0)
                    if(enable_flag):
                        self.__enable_flag = True
                    self.get_logger().info("--------------------")
                    # Check if the timeout has been exceeded
                    if elapsed_time > timeout:
                        self.get_logger().info("Timeout....")
                        elapsed_time_flag = True
                        enable_flag = True
                        break
                    time.sleep(1)
                    pass
            if(elapsed_time_flag):
                self.get_logger().info("Automatic enable timeout, exiting program")
                exit(0)

            self.PublishArmState()
            self.PublishArmJointAndGripper()
            self.PublishArmCtrlAndGripper()
            self.PublishArmEndPose()

            rate.sleep()

    def PublishArmState(self):
        arm_status = PiperStatusMsg()
        arm_status.ctrl_mode = self.piper.GetArmStatus().arm_status.ctrl_mode
        arm_status.arm_status = self.piper.GetArmStatus().arm_status.arm_status
        arm_status.mode_feedback = self.piper.GetArmStatus().arm_status.mode_feed
        arm_status.teach_status = self.piper.GetArmStatus().arm_status.teach_status
        arm_status.motion_status = self.piper.GetArmStatus().arm_status.motion_status
        arm_status.trajectory_num = self.piper.GetArmStatus().arm_status.trajectory_num
        arm_status.err_code = self.piper.GetArmStatus().arm_status.err_code
        arm_status.joint_1_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_1_angle_limit
        arm_status.joint_2_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_2_angle_limit
        arm_status.joint_3_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_3_angle_limit
        arm_status.joint_4_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_4_angle_limit
        arm_status.joint_5_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_5_angle_limit
        arm_status.joint_6_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_6_angle_limit
        arm_status.communication_status_joint_1 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_1
        arm_status.communication_status_joint_2 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_2
        arm_status.communication_status_joint_3 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_3
        arm_status.communication_status_joint_4 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_4
        arm_status.communication_status_joint_5 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_5
        arm_status.communication_status_joint_6 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_6
        self.arm_status_pub.publish(arm_status)

    def PublishArmJointAndGripper(self):
        # Assign timestamp
        self.joint_states.header.stamp = self.get_clock().now().to_msg()
        # Here, you can set the joint positions to any value you want
        # The raw data obtained is in degrees multiplied by 1000. To convert to radians, divide by 1000, multiply by π/180, and limit to 5 decimal places
        joint_0: float = (self.piper.GetArmJointMsgs().joint_state.joint_1 / 1000) * 0.017444
        joint_1: float = (self.piper.GetArmJointMsgs().joint_state.joint_2 / 1000) * 0.017444
        joint_2: float = (self.piper.GetArmJointMsgs().joint_state.joint_3 / 1000) * 0.017444
        joint_3: float = (self.piper.GetArmJointMsgs().joint_state.joint_4 / 1000) * 0.017444
        joint_4: float = (self.piper.GetArmJointMsgs().joint_state.joint_5 / 1000) * 0.017444
        joint_5: float = (self.piper.GetArmJointMsgs().joint_state.joint_6 / 1000) * 0.017444
        joint_6: float = self.piper.GetArmGripperMsgs().gripper_state.grippers_angle / 1000000
        vel_0: float = self.piper.GetArmHighSpdInfoMsgs().motor_1.motor_speed / 1000
        vel_1: float = self.piper.GetArmHighSpdInfoMsgs().motor_2.motor_speed / 1000
        vel_2: float = self.piper.GetArmHighSpdInfoMsgs().motor_3.motor_speed / 1000
        vel_3: float = self.piper.GetArmHighSpdInfoMsgs().motor_4.motor_speed / 1000
        vel_4: float = self.piper.GetArmHighSpdInfoMsgs().motor_5.motor_speed / 1000
        vel_5: float = self.piper.GetArmHighSpdInfoMsgs().motor_6.motor_speed / 1000
        effort_0:float = self.piper.GetArmHighSpdInfoMsgs().motor_1.effort/1000
        effort_1:float = self.piper.GetArmHighSpdInfoMsgs().motor_2.effort/1000
        effort_2:float = self.piper.GetArmHighSpdInfoMsgs().motor_3.effort/1000
        effort_3:float = self.piper.GetArmHighSpdInfoMsgs().motor_4.effort/1000
        effort_4:float = self.piper.GetArmHighSpdInfoMsgs().motor_5.effort/1000
        effort_5:float = self.piper.GetArmHighSpdInfoMsgs().motor_6.effort/1000
        effort_6:float = self.piper.GetArmGripperMsgs().gripper_state.grippers_effort/1000
        self.joint_states.position = [joint_0,joint_1, joint_2, joint_3, joint_4, joint_5,joint_6]
        self.joint_states.velocity = [vel_0, vel_1, vel_2, vel_3, vel_4, vel_5]
        self.joint_states.effort = [effort_0, effort_1, effort_2, effort_3, effort_4, effort_5, effort_6]
        # 发布所有消息
        self.joint_pub.publish(self.joint_states)

    def PublishArmCtrlAndGripper(self):
        self.joint_ctrl.header.stamp = self.get_clock().now().to_msg()
        joint_0: float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_1/1000) * 0.017444
        joint_1: float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_2/1000) * 0.017444
        joint_2: float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_3/1000) * 0.017444
        joint_3: float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_4/1000) * 0.017444
        joint_4: float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_5/1000) * 0.017444
        joint_5: float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_6/1000) * 0.017444
        joint_6: float = self.piper.GetArmGripperCtrl().gripper_ctrl.grippers_angle/1000000
        self.joint_ctrl.position = [joint_0, joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]  # Example values
        self.joint_ctrl_pub.publish(self.joint_ctrl)

    def PublishArmEndPose(self):
        # End effector pose
        endpos = Pose()
        end_pose = self.piper.GetArmEndPoseMsgs().end_pose
        endpos.position.x = end_pose.X_axis / 1000000
        endpos.position.y = end_pose.Y_axis / 1000000
        endpos.position.z = end_pose.Z_axis / 1000000
        roll = end_pose.RX_axis / 1000
        pitch = end_pose.RY_axis / 1000
        yaw = end_pose.RZ_axis / 1000
        quaternion = R.from_euler('xyz', [roll, pitch, yaw], degrees=True).as_quat()
        endpos.orientation.x = quaternion[0]
        endpos.orientation.y = quaternion[1]
        endpos.orientation.z = quaternion[2]
        endpos.orientation.w = quaternion[3]
        self.end_pose_pub.publish(endpos)

    def pos_callback(self, pose_msg):
        """Callback for `pos_command` (geometry_msgs/Pose) — Cartesian target
        for the end effector, expressed in arm/base_link. Orientation is a
        quaternion; we convert to XYZ Euler angles here because the Piper
        SDK's EndPoseCtrl takes rpy in 0.001 deg.

        Gripper is intentionally NOT part of this contract — command the
        gripper as a named joint on `joint_command`.
        """
        factor = 180 / 3.1415926
        px = pose_msg.position.x
        py = pose_msg.position.y
        pz = pose_msg.position.z
        qx = pose_msg.orientation.x
        qy = pose_msg.orientation.y
        qz = pose_msg.orientation.z
        qw = pose_msg.orientation.w
        # Guard against an uninitialised quaternion (all zeros) which would
        # blow up scipy's Rotation.from_quat — fall back to identity.
        if qx == 0.0 and qy == 0.0 and qz == 0.0 and qw == 0.0:
            roll = pitch = yaw = 0.0
        else:
            roll, pitch, yaw = R.from_quat([qx, qy, qz, qw]).as_euler('xyz', degrees=False)
        self.get_logger().info(
            f"Received pos_command: x={px:.4f} y={py:.4f} z={pz:.4f} "
            f"rpy=({roll:.4f}, {pitch:.4f}, {yaw:.4f})"
        )
        x = round(px * 1000) * 1000
        y = round(py * 1000) * 1000
        z = round(pz * 1000) * 1000
        rx = round(roll * 1000 * factor)
        ry = round(pitch * 1000 * factor)
        rz = round(yaw * 1000 * factor)
        if self.GetEnableFlag():
            self.piper.MotionCtrl_2(0x01, 0x00, 50)
            self.piper.EndPoseCtrl(x, y, z, rx, ry, rz)

    def joint_callback(self, joint_data):
        """Callback function for joint angles

        Args:
            joint_data (): The joint data
        """
        factor = 57324.840764  # 1000*180/3.14
        # self.get_logger().info(f"Received Joint States:")

        # 创建一个字典来存储关节名称与位置的映射
        joint_positions = {}
        joint_6 = 0

        # 遍历joint_data.name来映射位置
        for idx, joint_name in enumerate(joint_data.name):
            # self.get_logger().info(f"{joint_name}: {joint_data.position[idx]}")
            joint_positions[joint_name] = round(joint_data.position[idx] * factor)
        
        # 获取第7个关节的位置
        if len(joint_data.position) >= 7:
            # self.get_logger().info(f"joint_7: {joint_data.position[6]}")
            joint_6 = round(joint_data.position[6] * 1000 * 1000)
            joint_6 = joint_6 * self.gripper_val_mutiple
            joint_6 = self._clip_gripper_for_sdk(joint_6)

        # 控制电机速度
        if self.GetEnableFlag():
            if joint_data.velocity != []:
                all_zeros = all(v == 0 for v in joint_data.velocity)
            else:
                all_zeros = True
            if not all_zeros:
                lens = len(joint_data.velocity)
                if lens == 7:
                    vel_all = clip(round(joint_data.velocity[6]), 1, 100)
                    self.piper.MotionCtrl_2(0x01, 0x01, vel_all)
                else:
                    self.piper.MotionCtrl_2(0x01, 0x01, 30)
            else:
                self.piper.MotionCtrl_2(0x01, 0x01, 30)

            # 使用关节名称来动态控制关节
            joint_positions = self._clip_joint_positions_for_sdk(joint_positions)
            self.piper.JointCtrl(
                joint_positions.get('joint1', 0),
                joint_positions.get('joint2', 0),
                joint_positions.get('joint3', 0),
                joint_positions.get('joint4', 0),
                joint_positions.get('joint5', 0),
                joint_positions.get('joint6', 0)
            )

            # 夹爪控制
            if self.gripper_exist:
                if len(joint_data.effort) >= 7:
                    gripper_effort = clip(joint_data.effort[6], 0.5, 3)
                    # self.get_logger().info(f"gripper_effort: {gripper_effort}")
                    if not math.isnan(gripper_effort):
                        gripper_effort = round(gripper_effort * 1000)
                    else:
                        # self.get_logger().warning("Gripper effort is NaN, using default value.")
                        gripper_effort = 1000  # 设置默认值
                    self.piper.GripperCtrl(abs(joint_6), gripper_effort, 0x01, 0)
                else:
                    self.piper.GripperCtrl(abs(joint_6), 1000, 0x01, 0)


    def enable_callback(self, enable_flag: Bool):
        """Callback function for enabling the robotic arm

        Args:
            enable_flag (): Boolean flag
        """
        self.get_logger().info(f"Received enable flag:")
        self.get_logger().info(f"enable_flag: {enable_flag.data}")
        if enable_flag.data:
            self.__enable_flag = True
            self.piper.EnableArm(7)
            if self.gripper_exist:
                self.piper.GripperCtrl(0, 1000, 0x02, 0)
                self.piper.GripperCtrl(0, 1000, 0x01, 0)
        else:
            self.__enable_flag = False
            self.piper.DisableArm(7)
            if self.gripper_exist:
                self.piper.GripperCtrl(0, 1000, 0x02, 0)

    def handle_enable_service(self, req, resp):
        """Handle enable service for the robotic arm"""
        self.get_logger().info(f"Received request: {req.enable_request}")
        enable_flag = False
        loop_flag = False
        # Set timeout duration (seconds)
        timeout = 5
        # Record the time before entering the loop
        start_time = time.time()
        while not loop_flag:
            elapsed_time = time.time() - start_time
            self.get_logger().info(f"--------------------")
            enable_list = []
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status)
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status)
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status)
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status)
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status)
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status)

            if req.enable_request:
                enable_flag = all(enable_list)
                self.piper.EnableArm(7)
                self.piper.GripperCtrl(0, 1000, 0x01, 0)
            else:
                enable_flag = any(enable_list)
                self.piper.DisableArm(7)
                self.piper.GripperCtrl(0, 1000, 0x02, 0)

            self.get_logger().info(f"Enable status: {enable_flag}")
            self.__enable_flag = enable_flag
            self.get_logger().info(f"--------------------")

            if enable_flag == req.enable_request:
                loop_flag = True
                enable_flag = True
            else:
                loop_flag = False
                enable_flag = False

            # Check if timeout duration has been exceeded
            if elapsed_time > timeout:
                self.get_logger().info(f"Timeout...")
                enable_flag = False
                loop_flag = True
                break

            time.sleep(0.5)

        resp.enable_response = enable_flag
        self.get_logger().info(f"Returning response: {resp.enable_response}")
        return resp


def main(args=None):
    rclpy.init(args=args)
    piper_single_node = PiperRosNode()
    try:
        rclpy.spin(piper_single_node)
    except KeyboardInterrupt:
        pass
    finally:
        piper_single_node.destroy_node()
        rclpy.shutdown()
