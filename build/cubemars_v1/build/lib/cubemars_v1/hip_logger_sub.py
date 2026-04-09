#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
import csv
import os
import time
from datetime import datetime


class HipLogger(Node):

    def __init__(self):
        super().__init__('hip_logger')
        self.declare_parameter('log_root_dir', os.path.expanduser('~/exoskeleton_logs/hip'))
        self.declare_parameter('run_name_pattern', 'hip_run_{date}_{time}')
        self.declare_parameter('csv_filename', 'hip_log.csv')

        self.sub_cont = self.create_subscription(
            Float32MultiArray,
            '/hip_continous_data',
            self.cont_callback,
            10
        )

        self.sub_const = self.create_subscription(
            Float32,
            '/hip_constant_data',
            self.const_callback,
            10
        )
        self.sub_encoder = self.create_subscription(
            Float32,
            'current_angle',
            self.enc_callback,
            10
        )

      
        self.hip_cont = None
        self.hip_const = None
        self.enc_angle=None
        self.csv_angle=None
        self.row_counter = 0
        self.file_path = self._build_log_file_path()

        if not os.path.isfile(self.file_path) or os.path.getsize(self.file_path) == 0:
            with open(self.file_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['row_counter', 'time', 'hip_cont_angle','encoder_hip_angle','csv_hip_angle'])

    
        self.file = open(self.file_path, 'a', newline='')
        self.writer = csv.writer(self.file)

    
        self.start_time = time.time()

        self.timer = self.create_timer(0.03, self.log_data)
        self.get_logger().info(f"Logging to: {self.file_path}")

    def _build_log_file_path(self):
        root_dir = self.get_parameter('log_root_dir').get_parameter_value().string_value
        run_name_pattern = self.get_parameter('run_name_pattern').get_parameter_value().string_value
        csv_filename = self.get_parameter('csv_filename').get_parameter_value().string_value

        now = datetime.now()
        run_name = run_name_pattern.format(
            date=now.strftime('%Y%m%d'),
            time=now.strftime('%H%M%S')
        )

        run_dir = os.path.join(root_dir, run_name)
        suffix = 1
        while os.path.exists(run_dir):
            run_dir = os.path.join(root_dir, f"{run_name}_{suffix}")
            suffix += 1

        os.makedirs(run_dir, exist_ok=False)
        return os.path.join(run_dir, csv_filename)

 
    def cont_callback(self, msg):
        if len(msg.data) >= 2:
            self.hip_cont = msg.data[0]
            self.csv_angle = msg.data[1]

    def const_callback(self, msg):
        self.hip_const = msg.data
    def enc_callback(self,msg):
        self.enc_angle = self._normalize_deg(msg.data)

    @staticmethod
    def _normalize_deg(angle_deg: float) -> float:
        # Map any degree value to [-180, 180)
        return ((angle_deg + 180.0) % 360.0) - 180.0
    def log_data(self):
        current_time = time.time() - self.start_time
        if self.hip_const is not None or self.hip_cont is not None:
            self.writer.writerow([
                current_time,
                self.hip_cont,
                self.enc_angle,
                self.csv_angle
            ])

            self.file.flush()

    def destroy_node(self):
        self.file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = HipLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()