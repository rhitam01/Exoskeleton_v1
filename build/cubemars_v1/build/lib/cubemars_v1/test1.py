#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
import time
import can
import math
import logging
import os
import threading


class HipConstant(Node):

    def __init__(self):
        super().__init__('hip_constant')

        self.publisher_ = self.create_publisher(
            Float32,
            '/hip_constant_data',
            10
        )

        self.timer = self.create_timer(0.01, self.timer_callback)

        # -----------------------------
        # Motor configuration
        # -----------------------------
        self.MOTOR_ID = 0x03  # Hip only

        self.BITRATE = 1000000
        can.rc['interface'] = 'socketcan'
        can.rc['channel'] = 'can0'
        can.rc['bitrate'] = self.BITRATE

        self.bus = can.Bus()

        # MIT limits
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

        # Hip gains
        self.HIP_KP = 50
        self.HIP_KD = 5
        self.HIP_TORQ_FF = 1
        self.VEL_CMD = 0.01

        self.DT = 0.03

        threading.Thread(
            target=self.run_sequence,
            daemon=True
        ).start()

    # --------------------------------------------------

    def timer_callback(self):
        msg = Float32()

        feedback = self.receive_all()

        msg.data = feedback.get(self.MOTOR_ID, 0.0)

        self.publisher_.publish(msg)

    # --------------------------------------------------

    def receive_all(self):
        feedback = {}

        while True:
            msg = self.bus.recv(timeout=0.01)

            if msg is None:
                break

            motor_id, pos, vel, torque, temp = self.unpack_feedback(msg)

            feedback[motor_id] = pos

            print(
                f"ID: {motor_id} | "
                f"Pos: {math.degrees(pos):.2f} deg | "
                f"Vel: {vel:.2f} | "
                f"Torque: {torque:.2f} | "
                f"Temp: {temp}"
            )

        time.sleep(self.DT)

        return feedback

    # --------------------------------------------------

    def unpack_feedback(self, msg):
        data = msg.data

        motor_id = int(data[0])

        p_int = (data[1] << 8) | data[2]
        v_int = (data[3] << 4) | (data[4] >> 4)
        t_int = ((data[4] & 0xF) << 8) | data[5]
        temp = float(data[6]) - 40

        pos = self.uint_to_float(p_int, self.P_MIN, self.P_MAX, 16)
        vel = self.uint_to_float(v_int, self.V_MIN, self.V_MAX, 12)
        torque = self.uint_to_float(t_int, self.T_MIN, self.T_MAX, 12)

        return motor_id, pos, vel, torque, temp

    # --------------------------------------------------

    def uint_to_float(self, x_int, x_min, x_max, bits):
        return (
            float(x_int)
            * (x_max - x_min)
            / ((1 << bits) - 1)
            + x_min
        )

    def float_to_uint(self, x, x_min, x_max, bits):
        x = x_min if x < x_min else x_max if x > x_max else x

        return int(
            (x - x_min)
            * ((1 << bits) - 1)
            / (x_max - x_min)
        )

    # --------------------------------------------------

    def pack_cmd(self, p_des, v_des, kp, kd, t):
        p_des = min(max(self.P_MIN, p_des), self.P_MAX)
        v_des = min(max(self.V_MIN, v_des), self.V_MAX)
        kp = min(max(self.KP_MIN, kp), self.KP_MAX)
        kd = min(max(self.KD_MIN, kd), self.KD_MAX)
        t = min(max(self.T_MIN, t), self.T_MAX)

        p_int = self.float_to_uint(
            p_des, self.P_MIN, self.P_MAX, 16
        )
        v_int = self.float_to_uint(
            v_des, self.V_MIN, self.V_MAX, 12
        )
        kp_int = self.float_to_uint(
            kp, self.KP_MIN, self.KP_MAX, 12
        )
        kd_int = self.float_to_uint(
            kd, self.KD_MIN, self.KD_MAX, 12
        )
        t_int = self.float_to_uint(
            t, self.T_MIN, self.T_MAX, 12
        )

        return bytearray([
            p_int >> 8,
            p_int & 0xFF,
            v_int >> 4,
            ((v_int & 0xF) << 4) | (kp_int >> 8),
            kp_int & 0xFF,
            kd_int >> 4,
            ((kd_int & 0xF) << 4) | (t_int >> 8),
            t_int & 0xFF
        ])

    # --------------------------------------------------

    def enterMIT(self):
        self.bus.send(
            can.Message(
                arbitration_id=self.MOTOR_ID,
                data=[0xFF] * 7 + [0xFC],
                is_extended_id=False
            )
        )

    def setZero(self):
        self.bus.send(
            can.Message(
                arbitration_id=self.MOTOR_ID,
                data=[0xFF] * 7 + [0xFE],
                is_extended_id=False
            )
        )

    def exitMIT(self):
        self.bus.send(
            can.Message(
                arbitration_id=self.MOTOR_ID,
                data=[0xFF] * 7 + [0xFD],
                is_extended_id=False
            )
        )

    # --------------------------------------------------

    def send_command(self, target):
        self.bus.send(
            can.Message(
                arbitration_id=self.MOTOR_ID,
                data=self.pack_cmd(
                    target,
                    self.VEL_CMD,
                    self.HIP_KP,
                    self.HIP_KD,
                    self.HIP_TORQ_FF
                ),
                is_extended_id=False
            )
        )

    # --------------------------------------------------

    def run_sequence(self):

        self.enterMIT()
        time.sleep(0.05)

        self.setZero()
        time.sleep(0.05)

        for _ in range(10):
            self.bus.recv(timeout=0.01)

        home_pos = 0.0

        print("\nCapturing HOME position...\n")

        for _ in range(20):
            feedback = self.receive_all()

            if self.MOTOR_ID in feedback:
                home_pos = feedback[self.MOTOR_ID]

        print(
            f"Hip HOME: {math.degrees(home_pos):.2f} deg"
        )

        self.timer.cancel()

        target_deg = float(
            input("Hip target (deg): ")
        )

        self.timer.reset()

        target = (
            home_pos
            + math.radians(target_deg)
        )

        log_file = "constant_hip_log.csv"

        logger = logging.getLogger("constant_log")
        logger.setLevel(logging.DEBUG)

        if (
            not os.path.isfile(log_file)
            or os.path.getsize(log_file) == 0
        ):
            with open(log_file, "w") as f:
                f.write("time,hip_angle_constant\n")

        fh = logging.FileHandler(log_file)
        fh.setFormatter(
            logging.Formatter(
                "%(time)s,%(hip_angle_constant)s"
            )
        )

        if not logger.hasHandlers():
            logger.addHandler(fh)

        print("\nMoving to target...\n")

        start = time.time()

        for _ in range(100):

            self.send_command(target)

            feedback = self.receive_all()

            actual = feedback.get(
                self.MOTOR_ID,
                0.0
            )

            logger.info(
                "",
                extra={
                    "time": time.time() - start,
                    "hip_angle_constant": actual,
                },
            )

        print("\nReturning home...\n")

        time.sleep(1)

        for _ in range(100):

            self.send_command(home_pos)

            self.receive_all()

        self.exitMIT()

        self.bus.shutdown()


def main(args=None):
    rclpy.init(args=args)

    node = HipConstant()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
