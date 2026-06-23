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

        self._lock = threading.Lock()
        self.latest_hip_torque = 0.0
        self.latest_hip_angle = 0.0
        self.have_hip_torque = False

        self.create_subscription(Float32, '/hip_sim_data', self.hip_torque_callback, 10)
        self.hip_feedback_pub = self.create_publisher(Float32, '/hip_feedback', 10)

        self.timer = self.create_timer(0.01, self.timer_callback)

        self.MOTOR_ID = 0x02  # hip

        can.rc['interface'] = 'socketcan'
        can.rc['channel'] = 'can0'
        can.rc['bitrate'] = 1000000
        self.bus = can.Bus()

        # AK80-64 v2 MIT protocol limits
        self.P_MIN  = -12.5;  self.P_MAX  = 12.5   # rad
        self.V_MIN  = -8.0;  self.V_MAX  = 8.0   # rad/s  ← fixed from ±8
        self.KP_MIN = 0;      self.KP_MAX = 500
        self.KD_MIN = 0;      self.KD_MAX = 5
        self.T_MIN  = -144.0; self.T_MAX  = 144.0  # Nm (per v2 datasheet)

        # Pure torque feedforward — no impedance
        self.HIP_KP  = 0.0
        self.HIP_KD  = 0.0
        self.POS_CMD = 0.0
        self.VEL_CMD = 0.0

        self.DT = 0.0096
        self._running = True

        threading.Thread(target=self.run_sequence, daemon=True).start()

    # ------------------------------------------------------------------ #
    #  ROS callbacks                                                       #
    # ------------------------------------------------------------------ #

    def hip_torque_callback(self, msg):
        with self._lock:
            self.latest_hip_torque = float(msg.data)
            if self.latest_hip_torque >= 60:
                self.latest_hip_torque = 60
            self.have_hip_torque = True

    def timer_callback(self):
        with self._lock:
            angle = self.latest_hip_angle
        msg = Float32()
        msg.data = float(angle)
        self.hip_feedback_pub.publish(msg)

    # ------------------------------------------------------------------ #
    #  CAN helpers                                                         #
    # ------------------------------------------------------------------ #

    def uint_to_float(self, x_int, x_min, x_max, bits):
        return float(x_int) * (x_max - x_min) / ((1 << bits) - 1) + x_min

    def float_to_uint(self, x, x_min, x_max, bits):
        x = max(x_min, min(x_max, x))
        return int((x - x_min) * ((1 << bits) - 1) / (x_max - x_min))

    def pack_cmd(self, p_des, v_des, kp, kd, torque_ff):
        p_int  = self.float_to_uint(p_des,     self.P_MIN,  self.P_MAX,  16)
        v_int  = self.float_to_uint(v_des,     self.V_MIN,  self.V_MAX,  12)
        kp_int = self.float_to_uint(kp,        self.KP_MIN, self.KP_MAX, 12)
        kd_int = self.float_to_uint(kd,        self.KD_MIN, self.KD_MAX, 12)
        t_int  = self.float_to_uint(torque_ff, self.T_MIN,  self.T_MAX,  12)

        return bytearray([
            p_int >> 8,
            p_int & 0xFF,
            v_int >> 4,
            ((v_int  & 0xF) << 4) | (kp_int >> 8),
            kp_int & 0xFF,
            kd_int >> 4,
            ((kd_int & 0xF) << 4) | (t_int >> 8),
            t_int & 0xFF,
        ])

    def unpack_feedback(self, msg):
        data     = msg.data
        motor_id = msg.arbitration_id

        p_int = (data[1] << 8) | data[2]
        v_int = (data[3] << 4) | (data[4] >> 4)
        t_int = ((data[4] & 0xF) << 8) | data[5]
        temp  = float(data[6]) - 40

        pos    = self.uint_to_float(p_int, self.P_MIN, self.P_MAX, 16)
        vel    = self.uint_to_float(v_int, self.V_MIN, self.V_MAX, 12)
        torque = self.uint_to_float(t_int, self.T_MIN, self.T_MAX, 12)
        #print(torque)

        return motor_id, pos, vel, torque, temp

    def receive_all(self):
        feedback = {}
        while True:
            msg = self.bus.recv(timeout=0.005)
            if msg is None:
                break
            motor_id, pos, _, _, _ = self.unpack_feedback(msg)
            feedback[motor_id] = pos
        return feedback

    def enterMIT(self):
        self.bus.send(can.Message(
            arbitration_id=self.MOTOR_ID,
            data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC],
            is_extended_id=False,
        ))

    def setZero(self):
        self.bus.send(can.Message(
            arbitration_id=self.MOTOR_ID,
            data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE],
            is_extended_id=False,
        ))

    def exitMIT(self):
        self.bus.send(can.Message(
            arbitration_id=self.MOTOR_ID,
            data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD],
            is_extended_id=False,
        ))

    def send_torque_command(self, torque_ff=None):
        with self._lock:
            if torque_ff is None:
                torque_ff = self.latest_hip_torque
        self.bus.send(can.Message(
            arbitration_id=self.MOTOR_ID,
            data=self.pack_cmd(
                self.POS_CMD, self.VEL_CMD,
                self.HIP_KP,  self.HIP_KD,
                torque_ff,
            ),
            is_extended_id=False,
        ))

    def run_sequence(self):
        try:
            self.enterMIT()
            time.sleep(0.05)

            self.setZero()
            time.sleep(0.05)

            # flush stale frames
            for _ in range(10):
                self.bus.recv(timeout=0.01)

            self.get_logger().info('Waiting for /hip_torque...')
            while rclpy.ok() and self._running:
                with self._lock:
                    ready = self.have_hip_torque
                if ready:
                    break
                time.sleep(0.01)

            self.get_logger().info('Torque topic active — sending feedforward commands.')
            while rclpy.ok() and self._running:
                self.send_torque_command()

                feedback = self.receive_all()
                if self.MOTOR_ID in feedback:
                    with self._lock:
                        self.latest_hip_angle = feedback[self.MOTOR_ID]

                time.sleep(self.DT)

        finally:
            self.get_logger().info('Shutting down — zeroing torque.')
            self.send_torque_command(0.0)
            time.sleep(0.05)
            self.exitMIT()
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
