#!/usr/bin/env python3

import threading
import time

import can
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class MatlabTorqueCtrl(Node):
    def __init__(self):
        super().__init__('matlab_torque_ctrl')

        self.latest_hip_torque = 0.0
        self.latest_knee_torque = 0.0
        self.latest_hip_angle = 0.0
        self.latest_knee_angle = 0.0
        self.have_hip_torque = False
        self.have_knee_torque = False

        self.create_subscription(Float32, '/hip_torque', self.hip_torque_callback, 10)
        self.create_subscription(Float32, '/knee_torque', self.knee_torque_callback, 10)
        self.hip_feedback_pub = self.create_publisher(Float32, 'hip_feedback', 10)
        self.knee_feedback_pub = self.create_publisher(Float32, 'knee_feedback', 10)

        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.MOTOR_ID_1 = 0x02  # hip
        self.MOTOR_ID_2 = 0x03  # knee
        self.MOTOR_IDS = [self.MOTOR_ID_1, self.MOTOR_ID_2]

        self.BITRATE = 1000000
        can.rc['interface'] = 'socketcan'
        can.rc['channel'] = 'can0'
        can.rc['bitrate'] = self.BITRATE
        self.bus = can.Bus()

        self.P_MIN = -12.5
        self.P_MAX = 12.5
        self.V_MIN = -8.0
        self.V_MAX = 8.0
        self.KP_MIN = 0
        self.KP_MAX = 500
        self.KD_MIN = 0
        self.KD_MAX = 5
        self.T_MIN = -144
        self.T_MAX = 144

        # Pure torque feedforward: no impedance control.
        self.HIP_KP = 0
        self.HIP_KD = 0
        self.KNEE_KP = 0
        self.KNEE_KD = 0
        self.POS_CMD = 0.0
        self.VEL_CMD = 0.0

        self.DT = 0.0096
        self._running = True

        threading.Thread(target=self.run_sequence, daemon=True).start()

    def hip_torque_callback(self, msg):
        self.latest_hip_torque = float(msg.data)
        self.have_hip_torque = True

    def knee_torque_callback(self, msg):
        self.latest_knee_torque = float(msg.data)
        self.have_knee_torque = True

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

        p_int = (data[1] << 8) | data[2]
        v_int = (data[3] << 4) | (data[4] >> 4)
        t_int = ((data[4] & 0xF) << 8) | data[5]
        temp = float(data[6]) - 40

        pos = self.uint_to_float(p_int, self.P_MIN, self.P_MAX, 16)
        vel = self.uint_to_float(v_int, self.V_MIN, self.V_MAX, 12)
        torque = self.uint_to_float(t_int, self.T_MIN, self.T_MAX, 12)

        return motor_id, pos, vel, torque, temp

    def uint_to_float(self, x_int, x_min, x_max, bits):
        return float(x_int) * (x_max - x_min) / ((1 << bits) - 1) + x_min

    def float_to_uint(self, x, x_min, x_max, bits):
        x = x_min if x < x_min else x_max if x > x_max else x
        return int((x - x_min) * ((1 << bits) - 1) / (x_max - x_min))

    def pack_cmd(self, p_des, v_des, kp, kd, t):
        p_des = min(max(self.P_MIN, p_des), self.P_MAX)
        v_des = min(max(self.V_MIN, v_des), self.V_MAX)
        kp = min(max(self.KP_MIN, kp), self.KP_MAX)
        kd = min(max(self.KD_MIN, kd), self.KD_MAX)
        t = min(max(self.T_MIN, t), self.T_MAX)

        p_int = self.float_to_uint(p_des, self.P_MIN, self.P_MAX, 16)
        v_int = self.float_to_uint(v_des, self.V_MIN, self.V_MAX, 12)
        kp_int = self.float_to_uint(kp, self.KP_MIN, self.KP_MAX, 12)
        kd_int = self.float_to_uint(kd, self.KD_MIN, self.KD_MAX, 12)
        t_int = self.float_to_uint(t, self.T_MIN, self.T_MAX, 12)

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

    def send_torque_command(self, motor_id, torque_ff):
        if motor_id == self.MOTOR_ID_1:
            kp, kd = self.HIP_KP, self.HIP_KD
        elif motor_id == self.MOTOR_ID_2:
            kp, kd = self.KNEE_KP, self.KNEE_KD
        else:
            kp, kd = self.HIP_KP, self.HIP_KD

        self.bus.send(can.Message(
            arbitration_id=motor_id,
            data=self.pack_cmd(self.POS_CMD, self.VEL_CMD, kp, kd, torque_ff),
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

            self.get_logger().info('Waiting for /hip_torque and /knee_torque...')
            while rclpy.ok() and self._running and (not self.have_hip_torque or not self.have_knee_torque):
                time.sleep(0.01)

            self.get_logger().info('Torque topics active. Sending feedforward torque commands.')
            while rclpy.ok() and self._running:
                self.send_torque_command(self.MOTOR_ID_1, self.latest_hip_torque)
                self.send_torque_command(self.MOTOR_ID_2, self.latest_knee_torque)

                feedback = self.receive_all()
                self.latest_hip_angle = feedback.get(self.MOTOR_ID_1, self.latest_hip_angle)
                self.latest_knee_angle = feedback.get(self.MOTOR_ID_2, self.latest_knee_angle)

                time.sleep(self.DT)

        finally:
            for m in self.MOTOR_IDS:
                self.send_torque_command(m, 0.0)
                time.sleep(0.01)
            for m in self.MOTOR_IDS:
                self.exitMIT(m)
            self.bus.shutdown()

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MatlabTorqueCtrl()
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
