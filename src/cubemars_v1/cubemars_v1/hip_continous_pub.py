import rclpy
from rclpy.node import Node

from std_msgs.msg import String,Float32MultiArray
import datetime
import time
import can
import math
import pandas as pd
import logging
import csv
import io
import os
import threading

class Hip_Continous(Node):

    def __init__(self):
        super().__init__('hip_continous')
        self.df = pd.read_excel('/home/rhitam/exoskeleton-v1/src/cubemars_v1/data/anikait_hip_knee_angles.xlsx')
        self.latest_hip_angle = 0.0
        self.latest_csv_angle=0.0
        self.time_series = self.df["Dynamic"].values
        self.hip_angles = self.df["Left Hip Angles"].values
        self.knee_angles = self.df["Left Knee Angles"].values
        self.publisher_ = self.create_publisher(Float32MultiArray, '/hip_continous_data', 10)
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.MOTOR_ID_1 = 0x02#hip
        self.MOTOR_ID_2 = 0x03#knee 
        self.MOTOR_IDS = [self.MOTOR_ID_1, self.MOTOR_ID_2]

        self.BITRATE = 1000000
        can.rc['interface'] = 'socketcan'
        can.rc['channel'] = 'can0'
        can.rc['bitrate'] = self.BITRATE
        self.bus = can.Bus()

        self.P_MIN =-12.5
        self.P_MAX = 12.5
        self.V_MIN =-8.0
        self.V_MAX = 8.0
        self.KP_MIN = 0
        self.KP_MAX = 500
        self.KD_MIN = 0
        self.KD_MAX = 5
        self.T_MIN = -144
        self.T_MAX = 144

        self.HIP_KP = 100
        self.HIP_KD = 5
        self.KNEE_KP = 50
        self.KNEE_KD = 4
        self.VEL_CMD = 0.0
        self.TORQ_FF = 0.0
        self.DT = 0.03
        threading.Thread(target=self.run_sequence, daemon=True).start()
    def timer_callback(self):
        msg = Float32MultiArray()
        msg.data = [self.latest_hip_angle,self.latest_csv_angle]
        self.publisher_.publish(msg)
    def receive_all(self):
        feedback = {}
        while True:
            msg = self.bus.recv(timeout=0.01)
            if msg is None:
                break
            motor_id, pos, vel, torque, temp = self.unpack_feedback(msg)
            feedback[motor_id] = pos
            print(f"ID: {motor_id} | Pos: {math.degrees(pos):.2f}")
        time.sleep(self.DT)
        return feedback
    def unpack_feedback(self,msg):
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
    def uint_to_float(self,x_int, x_min, x_max, bits):
        return float(x_int) * (x_max - x_min) / ((1 << bits) - 1) + x_min
    def float_to_uint(self,x, min, max, bits):
        x = min if x < min else max if x > max else x
        return int((x - min) * ((1 << bits) - 1) / (max - min))
    def pack_cmd(self,p_des, v_des, kp, kd, t):
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

    def enterMIT(self,motor_id):
        self.bus.send(can.Message(arbitration_id=motor_id, data=[0xFF]*7 + [0xFC], is_extended_id=False))

    def setZero(self,motor_id):
        self.bus.send(can.Message(arbitration_id=motor_id, data=[0xFF]*7 + [0xFE], is_extended_id=False))

    def exitMIT(self,motor_id):
        self.bus.send(can.Message(arbitration_id=motor_id, data=[0xFF]*7 + [0xFD], is_extended_id=False))

    def send_command(self,motor_id, target):
        if motor_id == self.MOTOR_ID_1:
            kp, kd = self.HIP_KP, self.HIP_KD
        elif motor_id == self.MOTOR_ID_2:
            kp, kd = self.KNEE_KP, self.KNEE_KD
        else:
            kp, kd = self.HIP_KP, self.HIP_KD
        self.bus.send(can.Message(arbitration_id=motor_id, data=self.pack_cmd(target, self.VEL_CMD, kp, kd, self.TORQ_FF), is_extended_id=False))
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

            self.get_logger().info("Capturing HOME")

            for _ in range(30):
                for m in self.MOTOR_IDS:
                    self.send_command(m, 0)

                feedback = self.receive_all()

                for k, v in feedback.items():
                    self.home_pos[k] = v

            start_time = time.time()

            for i in range(len(self.time_series)):

                hip_target = self.home_pos[self.MOTOR_ID_1] + math.radians(self.hip_angles[i])
                knee_target = self.home_pos[self.MOTOR_ID_2] + math.radians(self.knee_angles[i])
                self.latest_csv_angle=self.hip_angles[i]
                self.send_command(self.MOTOR_ID_1, hip_target)
                self.send_command(self.MOTOR_ID_2, knee_target)

                feedback = self.receive_all()
                self.latest_hip_angle = feedback.get(self.MOTOR_ID_1, 0.0)

                current_time = time.time() - start_time
                #self.log_data(current_time, hip_angle)

                while time.time() - start_time < self.time_series[i]:
                    pass

            self.get_logger().info("Returning home")

            for _ in range(100):
                for m in self.MOTOR_IDS:
                    self.send_command(m, self.home_pos.get(m, 0))
                self.receive_all()

        finally:
            for m in self.MOTOR_IDS:
                self.exitMIT(m)
            self.bus.shutdown()
    
def main(args=None):
    rclpy.init(args=args)

    hip_continous = Hip_Continous()

    rclpy.spin(hip_continous)
    hip_continous.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
