#!/usr/bin/env python3

import csv
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
from scipy import signal


class JointTrajectoryFreqAnalysis(Node):
    def __init__(self):
        super().__init__("joint_trajectory_freq_analysis")

        self.declare_parameter(
            "angles_file",
            "/home/rhitam/exoskeleton-v1/src/cubemars_v1/data/anikait_hip_knee_angles.xlsx",
        )
        self.declare_parameter("run_tag", "freq_analysis")
        self.declare_parameter("dt_s", 0.03)
        self.declare_parameter("can_channel", "can0")
        self.declare_parameter("can_bitrate", 1000000)
        self.declare_parameter("output_dir", os.path.expanduser("~/exoskeleton_logs/trajectory"))
        self.declare_parameter("gait_band_hz_min", 0.05)
        self.declare_parameter("gait_band_hz_max", 3.0)
        self.declare_parameter("error_peak_min_hz", 0.02)
        self.declare_parameter("tracking_err_ratio_db", -10.0)

        self.angles_file = str(self.get_parameter("angles_file").value)
        self.run_tag = str(self.get_parameter("run_tag").value)
        self.dt_s = float(self.get_parameter("dt_s").value)
        self.can_channel = str(self.get_parameter("can_channel").value)
        self.can_bitrate = int(self.get_parameter("can_bitrate").value)
        self.output_dir = str(self.get_parameter("output_dir").value)
        self.gait_band_hz_min = float(self.get_parameter("gait_band_hz_min").value)
        self.gait_band_hz_max = float(self.get_parameter("gait_band_hz_max").value)
        self.error_peak_min_hz = float(self.get_parameter("error_peak_min_hz").value)
        self.tracking_err_ratio_db = float(self.get_parameter("tracking_err_ratio_db").value)

        self.MOTOR_ID_1 = 0x02
        self.MOTOR_ID_2 = 0x03
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

    @staticmethod
    def _uniform_time_and_signals(t_log, cmd_deg, meas_deg):
        t = np.asarray(t_log, dtype=float)
        cmd = np.asarray(cmd_deg, dtype=float)
        meas = np.asarray(meas_deg, dtype=float)
        n = len(t)
        if n < 4:
            return None
        t0, t1 = t[0], t[-1]
        dt = (t1 - t0) / max(n - 1, 1)
        if dt <= 0:
            return None
        t_u = np.linspace(t0, t1, n)
        cmd_u = np.interp(t_u, t, cmd)
        meas_u = np.interp(t_u, t, meas)
        return dt, t_u, cmd_u, meas_u

    def _rfft_spectrum(self, x, dt):
        """Single-sided magnitude spectrum (linear, same units as x)."""
        n = len(x)
        fs = 1.0 / dt
        x = signal.detrend(np.asarray(x, dtype=float), type="linear")
        win = np.hanning(n)
        cg = np.sum(win) / n
        xw = x * win
        X = np.fft.rfft(xw)
        freqs = np.fft.rfftfreq(n, d=dt)
        mag = (2.0 / (n * cg)) * np.abs(X)
        mag[0] = np.abs(X[0]) / (n * cg)
        return freqs, mag, X

    def _dominant_peaks(self, freqs, mag, min_hz, max_peaks=5, min_rel=0.08):
        mask = freqs >= min_hz
        if not np.any(mask):
            return []
        f = freqs[mask]
        m = mag[mask]
        if len(m) < 3:
            return []
        height = np.max(m) * min_rel
        peaks, _ = signal.find_peaks(m, height=height, distance=max(1, len(m) // 200))
        order = np.argsort(m[peaks])[::-1]
        out = []
        for idx in order[:max_peaks]:
            out.append(float(f[peaks[idx]]))
        return out

    def _transfer_function_fft(self, cmd, meas, dt):
        n = len(cmd)
        cmd = signal.detrend(np.asarray(cmd, dtype=float), type="linear")
        meas = signal.detrend(np.asarray(meas, dtype=float), type="linear")
        win = np.hanning(n)
        cg = np.sum(win) / n
        X = np.fft.rfft(cmd * win)
        Y = np.fft.rfft(meas * win)
        freqs = np.fft.rfftfreq(n, d=dt)
        tol = 1e-9 * (np.max(np.abs(X)) + 1e-12)
        G = np.divide(Y, X, out=np.full_like(Y, np.nan, dtype=np.complex128), where=np.abs(X) > tol)
        gain_db = 20.0 * np.log10(np.abs(G) + 1e-15)
        phase_deg = np.degrees(np.angle(G))
        return freqs, G, gain_db, phase_deg

    def _bandwidth_minus3db(self, freqs, gain_db, f_min_hz=0.01):
        """First frequency above f_min where |G| drops 3 dB from low-frequency reference."""
        mask = (freqs >= f_min_hz) & np.isfinite(gain_db)
        if not np.any(mask):
            return float("nan")
        f = freqs[mask]
        g = gain_db[mask]
        ref = np.median(g[: max(3, len(g) // 20)])
        target = ref - 3.0
        above = np.where(g <= target)[0]
        if len(above) == 0:
            return float("nan")
        return float(f[above[0]])

    def _tracking_degradation_freq(self, freqs, mag_cmd, mag_err):
        """Smallest f > 0 where |E|/|Cmd| exceeds linear ratio from tracking_err_ratio_db."""
        ratio_lin = 10.0 ** (self.tracking_err_ratio_db / 20.0)
        mask = freqs > 0.01
        cmd = np.asarray(mag_cmd)
        err = np.asarray(mag_err)
        with np.errstate(divide="ignore", invalid="ignore"):
            rel = np.divide(err, cmd + 1e-15)
        bad = mask & np.isfinite(rel) & (rel > ratio_lin)
        idx = np.where(bad)[0]
        if idx.size == 0:
            return float("nan")
        return float(freqs[idx[0]])

    def _phase_at_frequency(self, freqs, phase_deg, f_target):
        if not np.isfinite(f_target) or f_target <= 0:
            return float("nan")
        idx = int(np.argmin(np.abs(freqs - f_target)))
        return float(phase_deg[idx])

    def frequency_domain_analysis(
        self,
        run_dir,
        t_log,
        hip_cmd_deg,
        hip_meas_deg,
        knee_cmd_deg,
        knee_meas_deg,
    ):
        hip_u = self._uniform_time_and_signals(t_log, hip_cmd_deg, hip_meas_deg)
        knee_u = self._uniform_time_and_signals(t_log, knee_cmd_deg, knee_meas_deg)
        if hip_u is None or knee_u is None:
            self.get_logger().warning("Not enough samples for frequency analysis.")
            return

        dt_h, _, h_cmd, h_meas = hip_u
        dt_k, _, k_cmd, k_meas = knee_u
        lines = []

        for name, dt, c, m in (
            ("hip", dt_h, h_cmd, h_meas),
            ("knee", dt_k, k_cmd, k_meas),
        ):
            err = c - m
            freqs_c, mag_c, _ = self._rfft_spectrum(c, dt)
            freqs_m, mag_m, _ = self._rfft_spectrum(m, dt)
            freqs_e, mag_e, _ = self._rfft_spectrum(err, dt)

            fig, ax = plt.subplots(figsize=(11, 4))
            ax.plot(freqs_c, mag_c, label="|FFT(cmd)|", alpha=0.9)
            ax.plot(freqs_m, mag_m, label="|FFT(meas)|", alpha=0.85)
            ax.plot(freqs_e, mag_e, label="|FFT(error)|", alpha=0.85)
            ax.set_xlabel("Frequency [Hz]")
            ax.set_ylabel("Magnitude (detrended, windowed)")
            ax.set_title(f"{name.capitalize()}: magnitude spectrum (cmd / meas / error)")
            ax.set_xlim(0, min(15.0, freqs_c[-1]))
            ax.grid(True, alpha=0.3)
            ax.legend()
            plt.tight_layout()
            p_mag = os.path.join(run_dir, f"freq_magnitude_spectrum_{name}.png")
            plt.savefig(p_mag, dpi=150)
            plt.close()

            dom_err = self._dominant_peaks(freqs_e, mag_e, self.error_peak_min_hz)
            gait_mask = (freqs_c >= self.gait_band_hz_min) & (freqs_c <= self.gait_band_hz_max)
            if np.any(gait_mask):
                imax = np.argmax(mag_c[gait_mask])
                f_gait = float(freqs_c[gait_mask][imax])
            else:
                f_gait = dom_err[0] if dom_err else float("nan")

            freqs_tf, G, gain_db, phase_deg = self._transfer_function_fft(c, m, dt)
            bw = self._bandwidth_minus3db(freqs_tf, gain_db)
            f_track = self._tracking_degradation_freq(freqs_c, mag_c, mag_e)
            phase_gait = self._phase_at_frequency(freqs_tf, phase_deg, f_gait)
            phase_lag_deg = -phase_gait if np.isfinite(phase_gait) else float("nan")

            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
            ax1.plot(freqs_tf, gain_db, color="C0")
            ax1.axvline(f_gait, color="gray", ls="--", alpha=0.7, label=f"gait peak ~ {f_gait:.3f} Hz")
            ax1.set_ylabel("Gain [dB]")
            ax1.set_title(f"{name.capitalize()}: estimated G(jω) = FFT(meas)/FFT(cmd)")
            ax1.grid(True, alpha=0.3)
            ax1.legend()
            ax2.plot(freqs_tf, phase_deg, color="C1")
            ax2.set_xlabel("Frequency [Hz]")
            ax2.set_ylabel("Phase [deg]")
            ax2.grid(True, alpha=0.3)
            plt.tight_layout()
            p_tf = os.path.join(run_dir, f"freq_transfer_function_{name}.png")
            plt.savefig(p_tf, dpi=150)
            plt.close()

            self.get_logger().info(f"Saved magnitude spectrum: {p_mag}")
            self.get_logger().info(f"Saved transfer estimate plot: {p_tf}")

            lines.append(f"\n=== {name.upper()} ===\n")
            lines.append(f"Dominant error frequencies (Hz, top peaks): {dom_err}\n")
            lines.append(f"Estimated -3 dB bandwidth from |G| (Hz): {bw:.4f}\n")
            lines.append(f"Tracking degradation freq (|E|/|Cmd| > {self.tracking_err_ratio_db} dB): {f_track}\n")
            lines.append(f"Dominant gait band command peak (Hz): {f_gait:.4f}\n")
            lines.append(f"Phase of G = meas/cmd at gait peak (deg): {phase_gait:.2f}\n")
            lines.append(f"Phase lag estimate = -phase(G) at gait peak (deg): {phase_lag_deg:.2f}\n")

            print(f"\n--- {name.upper()} frequency analysis ---")
            print(f"Dominant error frequencies (Hz): {dom_err}")
            print(f"Estimated bandwidth (-3 dB, Hz): {bw}")
            print(f"Tracking degradation frequency (Hz): {f_track}")
            print(f"Dominant gait / cmd peak in band [{self.gait_band_hz_min}, {self.gait_band_hz_max}] Hz: {f_gait}")
            print(f"Phase of G at gait peak (deg): {phase_gait}")
            print(f"Phase lag estimate at gait peak (deg): {phase_lag_deg}")

        report_path = os.path.join(run_dir, "frequency_analysis_report.txt")
        with open(report_path, "w") as f:
            f.write("Sampling: uniform time grid from t_log endpoints; dt = (t_end - t_start) / (N-1).\n")
            f.write("FFT: linear detrend, Hann window, single-sided magnitude scaling.\n")
            f.write("G(jω): complex ratio FFT(meas)/FFT(cmd) with guard on small |FFT(cmd)|.\n")
            f.writelines(lines)
        self.get_logger().info(f"Saved report: {report_path}")

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
            self.frequency_domain_analysis(
                run_dir,
                t_log,
                hip_cmd_deg,
                hip_meas_deg,
                knee_cmd_deg,
                knee_meas_deg,
            )
            self.get_logger().info("Trajectory and frequency analysis complete.")

        finally:
            for motor_id in self.MOTOR_IDS:
                self.exit_mit(motor_id)
            self.bus.shutdown()
            if rclpy.ok():
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectoryFreqAnalysis()
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
