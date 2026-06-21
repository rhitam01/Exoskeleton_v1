#!/usr/bin/env python3

import math
import threading
import time

import can
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray


class HipKneeFromTopics(Node):
    def __init__(self):
        super().__init__('hip_knee_from_topics')

        self.latest_hip_angle = 0.0
        self.latest_knee_angle = 0.0
        self.latest_topic_hip_angle = 0.0
        self.latest_topic_knee_angle = 0.0
        self.have_hip = False
        self.have_knee = False

        self.hip_feedback_pub = self.create_publisher(Float32, 'hip_feedback', 10)
        self.knee_feedback_pub = self.create_publisher(Float32, 'knee_feedback', 10)
        self.create_subscription(Float32, '/hip_sim_data', self.hip_callback, 10)
        self.create_subscription(Float32, '/knee_sim_angle', self.knee_callback, 10)

        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.MOTOR_ID_1 = 0x02  # hip   -> CubeMars V2
        self.MOTOR_ID_2 = 0x03  # knee  -> CubeMars V1
        self.MOTOR_IDS = [self.MOTOR_ID_1, self.MOTOR_ID_2]

        self.BITRATE = 1000000
        can.rc['interface'] = 'socketcan'
        can.rc['channel'] = 'can0'
        can.rc['bitrate'] = self.BITRATE
        self.bus = can.Bus()

        # ---------------------------------------------------------------
        # Per-motor scaling parameters.
        # MOTOR_ID_1 (hip)  -> CubeMars V2 MIT-mode scaling
        # MOTOR_ID_2 (knee) -> CubeMars V1 MIT-mode scaling
        # Bit layout (pos16 | vel12 | kp12 | kd12 | torq12) is identical
        # between V1 and V2; only the physical min/max ranges differ.
        # ---------------------------------------------------------------
        self.MOTOR_PARAMS = {
            self.MOTOR_ID_1: {
                'P_MIN': -12.5, 'P_MAX': 12.5,
                'V_MIN': -8.0,  'V_MAX': 8.0,
                'KP_MIN': 0,    'KP_MAX': 500,
                'KD_MIN': 0,    'KD_MAX': 5,
                'T_MIN': -144,  'T_MAX': 144,
            },
            self.MOTOR_ID_2: {
                'P_MIN': -12.5, 'P_MAX': 12.5,
                'V_MIN': -46.57,   'V_MAX': 46.57,
                'KP_MIN': 0,       'KP_MAX': 500,
                'KD_MIN': 0,       'KD_MAX': 5,
                'T_MIN': -54,      'T_MAX': 54,
            },
        }

        self.HIP_KP = 40
        self.HIP_KD = 5
        self.KNEE_KP = 40
        self.KNEE_KD = 4
        self.VEL_CMD = 0.0
        self.TORQ_FF = 0.0

        self.DT = 0.0096
        self._running = True

        threading.Thread(target=self.run_sequence, daemon=True).start()

    def hip_callback(self, msg):
        # Topic is expected in radians.
        self.latest_topic_hip_angle = float(msg.data)
        self.have_hip = True

    def knee_callback(self, msg):
        # Topic is expected in radians.
        self.latest_topic_knee_angle = float(msg.data)
        self.have_knee = True

    def timer_callback(self):
        hip_msg = Float32()
        hip_msg.data = float(self.latest_hip_angle)
        self.hip_feedback_pub.publish(hip_msg)

        knee_msg = Float32()
        knee_msg.data = float(self.latest_knee_angle)
        self.knee_feedback_pub.publish(knee_msg)

    def receive_all(self):
        feedback = {}
        while True:
            msg = self.bus.recv(timeout=0.005)
            if msg is None:
                break
            motor_id, pos, _, _, _ = self.unpack_feedback(msg)
            feedback[motor_id] = pos
        return feedback

    def unpack_feedback(self, msg):
        data = msg.data
        motor_id = msg.arbitration_id
        params = self.MOTOR_PARAMS.get(motor_id, self.MOTOR_PARAMS[self.MOTOR_ID_1])

        p_int = (data[1] << 8) | data[2]
        v_int = (data[3] << 4) | (data[4] >> 4)
        t_int = ((data[4] & 0xF) << 8) | data[5]
        temp = float(data[6]) - 40

        pos = self.uint_to_float(p_int, params['P_MIN'], params['P_MAX'], 16)
        vel = self.uint_to_float(v_int, params['V_MIN'], params['V_MAX'], 12)
        torque = self.uint_to_float(t_int, params['T_MIN'], params['T_MAX'], 12)

        return motor_id, pos, vel, torque, temp

    def uint_to_float(self, x_int, x_min, x_max, bits):
        return float(x_int) * (x_max - x_min) / ((1 << bits) - 1) + x_min

    def float_to_uint(self, x, x_min, x_max, bits):
        x = x_min if x < x_min else x_max if x > x_max else x
        return int((x - x_min) * ((1 << bits) - 1) / (x_max - x_min))

    def pack_cmd(self, motor_id, p_des, v_des, kp, kd, t):
        params = self.MOTOR_PARAMS.get(motor_id, self.MOTOR_PARAMS[self.MOTOR_ID_1])

        p_des = min(max(params['P_MIN'], p_des), params['P_MAX'])
        v_des = min(max(params['V_MIN'], v_des), params['V_MAX'])
        kp = min(max(params['KP_MIN'], kp), params['KP_MAX'])
        kd = min(max(params['KD_MIN'], kd), params['KD_MAX'])
        t = min(max(params['T_MIN'], t), params['T_MAX'])

        p_int = self.float_to_uint(p_des, params['P_MIN'], params['P_MAX'], 16)
        v_int = self.float_to_uint(v_des, params['V_MIN'], params['V_MAX'], 12)
        kp_int = self.float_to_uint(kp, params['KP_MIN'], params['KP_MAX'], 12)
        kd_int = self.float_to_uint(kd, params['KD_MIN'], params['KD_MAX'], 12)
        t_int = self.float_to_uint(t, params['T_MIN'], params['T_MAX'], 12)

        return bytearray([
            p_int >> 8, p_int & 0xFF,
            v_int >> 4,
            ((v_int & 0xF) << 4) | (kp_int >> 8),
            kp_int & 0xFF,
            kd_int >> 4,
            ((kd_int & 0xF) << 4) | (t_int >> 8),
            t_int & 0xFF
        ])

    def enterMIT(self, motor_id):
        self.bus.send(can.Message(arbitration_id=motor_id, data=[0xFF] * 7 + [0xFC], is_extended_id=False))

    def setZero(self, motor_id):
        self.bus.send(can.Message(arbitration_id=motor_id, data=[0xFF] * 7 + [0xFE], is_extended_id=False))

    def exitMIT(self, motor_id):
        self.bus.send(can.Message(arbitration_id=motor_id, data=[0xFF] * 7 + [0xFD], is_extended_id=False))

    def send_command(self, motor_id, target):
        if motor_id == self.MOTOR_ID_1:
            kp, kd = self.HIP_KP, self.HIP_KD
        elif motor_id == self.MOTOR_ID_2:
            kp, kd = self.KNEE_KP, self.KNEE_KD
        else:
            kp, kd = self.HIP_KP, self.HIP_KD
        self.bus.send(can.Message(
            arbitration_id=motor_id,
            data=self.pack_cmd(motor_id, target, self.VEL_CMD, kp, kd, self.TORQ_FF),
            is_extended_id=False
        ))

    def run_sequence(self):
        try:
            for m in self.MOTOR_IDS:
                self.enterMIT(m)
                time.sleep(0.05)

            for m in self.MOTOR_IDS:
                self.setZero(m)
                time.sleep(0.05)

            for _ in range(10):
                self.bus.recv(timeout=0.01)

            self.home_pos = {}
            self.get_logger().info('Capturing HOME')

            for _ in range(30):
                for m in self.MOTOR_IDS:
                    self.send_command(m, 0)
                feedback = self.receive_all()
                for k, v in feedback.items():
                    self.home_pos[k] = v
                time.sleep(self.DT)

            hip_home = self.home_pos.get(self.MOTOR_ID_1, 0.0)
            knee_home = self.home_pos.get(self.MOTOR_ID_2, 0.0)

            self.get_logger().info('Waiting for /hip_sim_data and /knee_sim_angle...')
            while rclpy.ok() and self._running and (not self.have_hip or not self.have_knee):
                time.sleep(0.01)

            self.get_logger().info('Topics active. Streaming commands from topic angles.')
            while rclpy.ok() and self._running:
                hip_target = hip_home + self.latest_topic_hip_angle
                knee_target = knee_home + self.latest_topic_knee_angle

                self.send_command(self.MOTOR_ID_1, hip_target)
                self.send_command(self.MOTOR_ID_2, knee_target)

                feedback = self.receive_all()
                hip_fb_rad = feedback.get(self.MOTOR_ID_1, hip_target)
                knee_fb_rad = feedback.get(self.MOTOR_ID_2, knee_target)
                self.latest_hip_angle = hip_fb_rad
                self.latest_knee_angle = knee_fb_rad

                time.sleep(self.DT)

        finally:
            for m in self.MOTOR_IDS:
                self.exitMIT(m)
            self.bus.shutdown()

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HipKneeFromTopics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
