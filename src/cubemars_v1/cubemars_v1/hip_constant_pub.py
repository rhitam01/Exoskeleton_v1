import rclpy
from rclpy.node import Node

from std_msgs.msg import String,Float32
import datetime
import time
import can
import math
import pandas as pd
import logging
import csv
import os
import io
import threading

class Hip_Constant(Node):

    def __init__(self):
        super().__init__('hip_constant')
        self.publisher_ = self.create_publisher(Float32, '/hip_constant_data', 10)
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

        self.HIP_KP = 500
        self.HIP_KD = 5
        self.HIP_TORQ_FF = 2
#        self.KNEE_KP = 25
        self.KNEE_KP = 500
        self.KNEE_KD = 2
        self.VEL_CMD = 0.01
        self.KNEE_TORQ_FF = 0.5
        self.DT = 0.03
        threading.Thread(target=self.run_sequence, daemon=True).start()
    def timer_callback(self):
        msg = Float32()
        feedback = self.receive_all()
        msg.data = feedback.get(self.MOTOR_ID_1, 0.0)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
    def receive_all(self):
        """Read all available messages in buffer"""
        feedback = {}

        while True:
            msg = self.bus.recv(timeout=0.01)
            if msg is None:
                break

            motor_id, pos, vel, torque, temp = self.unpack_feedback(msg)

            feedback[motor_id] = pos

            print(f"ID: {motor_id} | Pos: {math.degrees(pos):.2f} deg | Vel: {vel:.2f} | Torque: {torque:.2f} | Temp: {temp}")

        time.sleep(self.DT)
        return feedback
    def unpack_feedback(self,msg):
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
        self.bus.send(can.Message(
            arbitration_id=motor_id,
            data=[0xFF]*7 + [0xFC],
            is_extended_id=False
        ))


    def setZero(self,motor_id):
        self.bus.send(can.Message(
            arbitration_id=motor_id,
            data=[0xFF]*7 + [0xFE],
            is_extended_id=False
        ))


    def exitMIT(self,motor_id):
        self.bus.send(can.Message(
            arbitration_id=motor_id,
            data=[0xFF]*7 + [0xFD],
            is_extended_id=False
        ))


    def send_command(self,motor_id, target):
        if motor_id == self.MOTOR_ID_1:
            kp, kd, tqff = self.HIP_KP, self.HIP_KD, self.HIP_TORQ_FF
        elif motor_id == self.MOTOR_ID_2:
            kp, kd, tqff = self.KNEE_KP, self.KNEE_KD, self.KNEE_TORQ_FF
        else:
            kp, kd, tqff = self.HIP_KP, self.HIP_KD, self.HIP_TORQ_FF
        self.bus.send(can.Message(
            arbitration_id=motor_id,
            data=self.pack_cmd(target, self.VEL_CMD, kp, kd, tqff),
            is_extended_id=False
        ))
    def run_sequence(self):

        for m in self.MOTOR_IDS:
            self.enterMIT(m)
            time.sleep(0.05)

        for m in self.MOTOR_IDS:
            self.setZero(m)
            time.sleep(0.05)

        for _ in range(10):
            self.bus.recv(timeout=0.01)

        home_pos = {}

        print("\nCapturing HOME positions...\n")

        for _ in range(20):
            feedback = self.receive_all()
            for k, v in feedback.items():
                home_pos[k] = v

        print("\nHOME POSITIONS:")
        for k, v in home_pos.items():
            print(f"Motor {k}: {math.degrees(v):.2f} deg")

        print("\nEnter target angles:\n")

        print("\nEnter target angles:\n")

        self.timer.cancel()  

        target_deg_1 = float(input("Motor 2 target (deg): "))
        target_deg_2 = float(input("Motor 3 target (deg): "))

        self.timer.reset()    

        targets = {
            self.MOTOR_ID_1: home_pos.get(self.MOTOR_ID_1, 0) + math.radians(target_deg_1),
            self.MOTOR_ID_2: home_pos.get(self.MOTOR_ID_2, 0) + math.radians(target_deg_2)
        }

        log_file = "constant_hip_log.csv"

        lgr = logging.getLogger("constant_log")
        lgr.setLevel(logging.DEBUG)

        if not os.path.isfile(log_file) or os.path.getsize(log_file) == 0:
            with open(log_file, 'w') as f:
                f.write('time,hip_angle_constant\n')

        fh = logging.FileHandler(log_file)
        frmt = logging.Formatter('%(time)s,%(hip_angle_constant)s')
        fh.setFormatter(frmt)

        if not lgr.hasHandlers():
            lgr.addHandler(fh)

        print("\nMoving to target...\n")

        start_time = time.time()   # ← ONLY small addition (required)

        for _ in range(100):
            for m in self.MOTOR_IDS:
                self.send_command(m, targets[m])

            current_time = time.time() - start_time

            feedback = self.receive_all()
            actual_angle = feedback.get(self.MOTOR_ID_1, 0.0)

            lgr.info('', extra={
                'time': current_time,
                'hip_angle_constant': actual_angle,
            })

        print("\nReturning to home...\n")

        time.sleep(1)

        for _ in range(100):
            for m in self.MOTOR_IDS:
                self.send_command(m, home_pos.get(m, 0))

            self.receive_all()

        for m in self.MOTOR_IDS:
            self.exitMIT(m)

        self.bus.shutdown()

    print("\nDone.")
def main(args=None):
    rclpy.init(args=args)

    hip_constant = Hip_Constant()

    rclpy.spin(hip_constant)
    hip_constant.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
