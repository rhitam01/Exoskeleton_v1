#!/usr/bin/env python3

import csv
import math
import os
import threading
import time
from datetime import datetime

import can
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import rclpy
from rclpy.node import Node


class JointTrajectoryExec(Node):
    def __init__(self):
        super().__init__("joint_trajectory_exec")

        self.declare_parameter(
            "angles_file",
            "/home/rhitam/exoskeleton-v1/src/cubemars_v1/data/anikait_hip_knee_angles.xlsx",
        )
        self.declare_parameter("run_tag", "with_trajectory")
        self.declare_parameter("dt_s", 0.03)
        self.declare_parameter("can_channel", "can0")
        self.declare_parameter("can_bitrate", 1000000)
        self.declare_parameter("output_dir", os.path.expanduser("~/exoskeleton_logs/trajectory"))

        self.angles_file = str(self.get_parameter("angles_file").value)
        self.run_tag = str(self.get_parameter("run_tag").value)
        self.dt_s = float(self.get_parameter("dt_s").value)
        self.can_channel = str(self.get_parameter("can_channel").value)
        self.can_bitrate = int(self.get_parameter("can_bitrate").value)
        self.output_dir = str(self.get_parameter("output_dir").value)

        self.MOTOR_ID_1 = 0x02  # hip
        self.MOTOR_ID_2 = 0x03  # knee
        self.MOTOR_IDS = [self.MOTOR_ID_1, self.MOTOR_ID_2]

        self.P_MIN = -12.5
        self.P_MAX = 12.5
        self.V_MIN = -8.0
        self.V_MAX = 8.0
        self.KP_MIN = 0.0
        self.KP_MAX = 500.0
        self.KD_MIN = 0.0
        self.KD_MAX = 5.0
        self.T_MIN = -144.0
        self.T_MAX = 144.0

        self.HIP_KP = 500.0
        self.HIP_KD = 5.0
        self.KNEE_KP = 500.0
        self.KNEE_KD = 5.0
        self.TORQ_FF = 0.0

        can.rc["interface"] = "socketcan"
        can.rc["channel"] = self.can_channel
        can.rc["bitrate"] = self.can_bitrate
        self.bus = can.Bus()

        self.time_series, self.hip_angles_deg, self.knee_angles_deg = self.load_source_angles()
        self.traj_t, self.hip_traj_deg, self.knee_traj_deg = self.generate_trajectory_from_file()

        self.exec_thread = threading.Thread(target=self.run_sequence, daemon=True)
        self.exec_thread.start()

    def load_source_angles(self):
        df = pd.read_excel(self.angles_file)
        time_series = df["Dynamic"].to_numpy(dtype=float)
        hip_angles = df["Left Hip Angles"].to_numpy(dtype=float)
        knee_angles = df["Left Knee Angles"].to_numpy(dtype=float)
        return time_series, hip_angles, knee_angles

    def generate_trajectory_from_file(self):
        t_src = np.asarray(self.time_series, dtype=float)
        hip_src = np.asarray(self.hip_angles_deg, dtype=float)
        knee_src = np.asarray(self.knee_angles_deg, dtype=float)

        if t_src.size < 2:
            t_uniform = np.array([0.0], dtype=float)
            return t_uniform, hip_src[:1], knee_src[:1]

        total_t = float(t_src[-1])
        t_uniform = np.arange(0.0, total_t + self.dt_s, self.dt_s, dtype=float)
        hip_interp = np.interp(t_uniform, t_src, hip_src)
        knee_interp = np.interp(t_uniform, t_src, knee_src)

        win = 14
        kernel = np.ones(win) / float(win)
        hip_smooth = np.convolve(hip_interp, kernel, mode="same")
        knee_smooth = np.convolve(knee_interp, kernel, mode="same")
        return t_uniform, hip_smooth, knee_smooth

    def float_to_uint(self, value, minimum, maximum, bits):
        value = minimum if value < minimum else maximum if value > maximum else value
        return int((value - minimum) * ((1 << bits) - 1) / (maximum - minimum))

    def uint_to_float(self, value_int, minimum, maximum, bits):
        return float(value_int) * (maximum - minimum) / ((1 << bits) - 1) + minimum

    def pack_cmd(self, p_des, v_des, kp, kd, torque):
        p_des = min(max(self.P_MIN, p_des), self.P_MAX)
        v_des = min(max(self.V_MIN, v_des), self.V_MAX)
        kp = min(max(self.KP_MIN, kp), self.KP_MAX)
        kd = min(max(self.KD_MIN, kd), self.KD_MAX)
        torque = min(max(self.T_MIN, torque), self.T_MAX)

        p_int = self.float_to_uint(p_des, self.P_MIN, self.P_MAX, 16)
        v_int = self.float_to_uint(v_des, self.V_MIN, self.V_MAX, 12)
        kp_int = self.float_to_uint(kp, self.KP_MIN, self.KP_MAX, 12)
        kd_int = self.float_to_uint(kd, self.KD_MIN, self.KD_MAX, 12)
        t_int = self.float_to_uint(torque, self.T_MIN, self.T_MAX, 12)

        return bytearray(
            [
                p_int >> 8,
                p_int & 0xFF,
                v_int >> 4,
                ((v_int & 0xF) << 4) | (kp_int >> 8),
                kp_int & 0xFF,
                kd_int >> 4,
                ((kd_int & 0xF) << 4) | (t_int >> 8),
                t_int & 0xFF,
            ]
        )

    def unpack_feedback(self, msg):
        data = msg.data
        motor_id = msg.arbitration_id
        p_int = (data[1] << 8) | data[2]
        v_int = (data[3] << 4) | (data[4] >> 4)
        t_int = ((data[4] & 0xF) << 8) | data[5]

        pos = self.uint_to_float(p_int, self.P_MIN, self.P_MAX, 16)
        vel = self.uint_to_float(v_int, self.V_MIN, self.V_MAX, 12)
        torque = self.uint_to_float(t_int, self.T_MIN, self.T_MAX, 12)
        return motor_id, pos, vel, torque

    def enter_mit(self, motor_id):
        self.bus.send(
            can.Message(arbitration_id=motor_id, data=[0xFF] * 7 + [0xFC], is_extended_id=False)
        )

    def set_zero(self, motor_id):
        self.bus.send(
            can.Message(arbitration_id=motor_id, data=[0xFF] * 7 + [0xFE], is_extended_id=False)
        )

    def exit_mit(self, motor_id):
        self.bus.send(
            can.Message(arbitration_id=motor_id, data=[0xFF] * 7 + [0xFD], is_extended_id=False)
        )

    def send_command(self, motor_id, pos_rad, vel_rad_s):
        if motor_id == self.MOTOR_ID_1:
            kp, kd = self.HIP_KP, self.HIP_KD
        else:
            kp, kd = self.KNEE_KP, self.KNEE_KD
        self.bus.send(
            can.Message(
                arbitration_id=motor_id,
                data=self.pack_cmd(pos_rad, vel_rad_s, kp, kd, self.TORQ_FF),
                is_extended_id=False,
            )
        )

    def receive_all(self):
        feedback = {}
        while True:
            msg = self.bus.recv(timeout=0.005)
            if msg is None:
                break
            motor_id, pos, vel, _ = self.unpack_feedback(msg)
            feedback[motor_id] = {"pos": pos, "vel": vel}
        return feedback

    @staticmethod
    def _gain_token(value):
        """Encode a gain for use in filenames (no dots or minus sign issues on all FS)."""
        t = f"{float(value):g}"
        return t.replace("-", "m").replace(".", "p")

    def _angles_csv_basename(self):
        hkp = self._gain_token(self.HIP_KP)
        hkd = self._gain_token(self.HIP_KD)
        kkp = self._gain_token(self.KNEE_KP)
        kkd = self._gain_token(self.KNEE_KD)
        return f"joint_angles_hipkp{hkp}_hipkd{hkd}_kneekp{kkp}_kneekd{kkd}.csv"

    def save_timeseries_and_plots(
        self,
        run_dir,
        t_log,
        hip_cmd_deg,
        knee_cmd_deg,
        hip_meas_deg,
        knee_meas_deg,
        hip_vel_cmd_deg_s,
        knee_vel_cmd_deg_s,
        hip_vel_meas_deg_s,
        knee_vel_meas_deg_s,
    ):
        csv_path = os.path.join(run_dir, "trajectory_timeseries.csv")
        with open(csv_path, "w", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(
                [
                    "time_s",
                    "hip_cmd_deg",
                    "knee_cmd_deg",
                    "hip_meas_deg",
                    "knee_meas_deg",
                    "hip_cmd_vel_deg_s",
                    "knee_cmd_vel_deg_s",
                    "hip_meas_vel_deg_s",
                    "knee_meas_vel_deg_s",
                ]
            )
            for i in range(len(t_log)):
                writer.writerow(
                    [
                        t_log[i],
                        hip_cmd_deg[i],
                        knee_cmd_deg[i],
                        hip_meas_deg[i],
                        knee_meas_deg[i],
                        hip_vel_cmd_deg_s[i],
                        knee_vel_cmd_deg_s[i],
                        hip_vel_meas_deg_s[i],
                        knee_vel_meas_deg_s[i],
                    ]
                )

        angles_csv_path = os.path.join(run_dir, self._angles_csv_basename())
        with open(angles_csv_path, "w", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(
                [
                    "time_s",
                    "hip_angle_cmd_deg",
                    "hip_angle_executed_deg",
                    "knee_angle_cmd_deg",
                    "knee_angle_executed_deg",
                ]
            )
            for i in range(len(t_log)):
                writer.writerow(
                    [
                        t_log[i],
                        hip_cmd_deg[i],
                        hip_meas_deg[i],
                        knee_cmd_deg[i],
                        knee_meas_deg[i],
                    ]
                )

        angle_path = os.path.join(run_dir, "joint_angle_graph.png")
        velocity_path = os.path.join(run_dir, "joint_velocity_graph.png")

        plt.figure(figsize=(11, 4))
        plt.plot(t_log, hip_cmd_deg, label="Hip cmd")
        plt.plot(t_log, hip_meas_deg, label="Hip meas", alpha=0.85)
        plt.plot(t_log, knee_cmd_deg, label="Knee cmd")
        plt.plot(t_log, knee_meas_deg, label="Knee meas", alpha=0.85)
        plt.xlabel("Time [s]")
        plt.ylabel("Angle [deg]")
        plt.title(f"Joint Angles vs Time ({self.run_tag})")
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.tight_layout()
        plt.savefig(angle_path, dpi=150)
        plt.close()

        plt.figure(figsize=(11, 4))
        plt.plot(t_log, hip_vel_cmd_deg_s, label="Hip cmd vel")
        plt.plot(t_log, hip_vel_meas_deg_s, label="Hip meas vel", alpha=0.85)
        plt.plot(t_log, knee_vel_cmd_deg_s, label="Knee cmd vel")
        plt.plot(t_log, knee_vel_meas_deg_s, label="Knee meas vel", alpha=0.85)
        plt.xlabel("Time [s]")
        plt.ylabel("Velocity [deg/s]")
        plt.title(f"Joint Velocities vs Time ({self.run_tag})")
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.tight_layout()
        plt.savefig(velocity_path, dpi=150)
        plt.close()

        self.get_logger().info(f"Saved CSV log: {csv_path}")
        self.get_logger().info(f"Saved angles CSV: {angles_csv_path}")
        self.get_logger().info(f"Saved angle plot: {angle_path}")
        self.get_logger().info(f"Saved velocity plot: {velocity_path}")

    def run_sequence(self):
        try:
            for motor_id in self.MOTOR_IDS:
                self.enter_mit(motor_id)
                time.sleep(0.05)
            for motor_id in self.MOTOR_IDS:
                self.set_zero(motor_id)
                time.sleep(0.05)

            home_pos = {}
            for _ in range(30):
                for motor_id in self.MOTOR_IDS:
                    self.send_command(motor_id, 0.0, 0.0)
                feedback = self.receive_all()
                for motor_id, values in feedback.items():
                    home_pos[motor_id] = values["pos"]
                time.sleep(self.dt_s)

            if self.MOTOR_ID_1 not in home_pos:
                home_pos[self.MOTOR_ID_1] = 0.0
            if self.MOTOR_ID_2 not in home_pos:
                home_pos[self.MOTOR_ID_2] = 0.0

            hip_ref_abs_rad = home_pos[self.MOTOR_ID_1] + np.radians(self.hip_traj_deg)
            knee_ref_abs_rad = home_pos[self.MOTOR_ID_2] + np.radians(self.knee_traj_deg)
            hip_vel_cmd = np.gradient(hip_ref_abs_rad, self.dt_s, edge_order=1)
            knee_vel_cmd = np.gradient(knee_ref_abs_rad, self.dt_s, edge_order=1)

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            run_dir = os.path.join(self.output_dir, f"{self.run_tag}_{timestamp}")
            os.makedirs(run_dir, exist_ok=True)

            t_log = []
            hip_cmd_deg = []
            knee_cmd_deg = []
            hip_meas_deg = []
            knee_meas_deg = []
            hip_vel_cmd_deg_s = []
            knee_vel_cmd_deg_s = []
            hip_vel_meas_deg_s = []
            knee_vel_meas_deg_s = []

            start = time.time()
            for i, t_target in enumerate(self.traj_t):
                self.send_command(self.MOTOR_ID_1, float(hip_ref_abs_rad[i]), float(hip_vel_cmd[i]))
                self.send_command(self.MOTOR_ID_2, float(knee_ref_abs_rad[i]), float(knee_vel_cmd[i]))

                fb = self.receive_all()
                hip_fb = fb.get(self.MOTOR_ID_1, {"pos": float(hip_ref_abs_rad[i]), "vel": float(hip_vel_cmd[i])})
                knee_fb = fb.get(self.MOTOR_ID_2, {"pos": float(knee_ref_abs_rad[i]), "vel": float(knee_vel_cmd[i])})

                t_now = time.time() - start
                t_log.append(t_now)
                hip_cmd_deg.append(float(np.degrees(hip_ref_abs_rad[i])))
                knee_cmd_deg.append(float(np.degrees(knee_ref_abs_rad[i])))
                hip_meas_deg.append(float(np.degrees(hip_fb["pos"])))
                knee_meas_deg.append(float(np.degrees(knee_fb["pos"])))
                hip_vel_cmd_deg_s.append(float(np.degrees(hip_vel_cmd[i])))
                knee_vel_cmd_deg_s.append(float(np.degrees(knee_vel_cmd[i])))
                hip_vel_meas_deg_s.append(float(np.degrees(hip_fb["vel"])))
                knee_vel_meas_deg_s.append(float(np.degrees(knee_fb["vel"])))

                sleep_time = t_target - (time.time() - start)
                if sleep_time > 0:
                    time.sleep(sleep_time)

            for _ in range(80):
                self.send_command(self.MOTOR_ID_1, home_pos[self.MOTOR_ID_1], 0.0)
                self.send_command(self.MOTOR_ID_2, home_pos[self.MOTOR_ID_2], 0.0)
                self.receive_all()
                time.sleep(self.dt_s)

            self.save_timeseries_and_plots(
                run_dir,
                t_log,
                hip_cmd_deg,
                knee_cmd_deg,
                hip_meas_deg,
                knee_meas_deg,
                hip_vel_cmd_deg_s,
                knee_vel_cmd_deg_s,
                hip_vel_meas_deg_s,
                knee_vel_meas_deg_s,
            )
            self.get_logger().info("Trajectory execution complete.")

        finally:
            for motor_id in self.MOTOR_IDS:
                self.exit_mit(motor_id)
            self.bus.shutdown()
            if rclpy.ok():
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectoryExec()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
