#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
import csv
import os
import time


class HipLogger(Node):

    def __init__(self):
        super().__init__('hip_logger')

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
        
        base_name = 'hip_log'
        extension = '.csv'
        counter = 0

        while True:
            if counter == 0:
                file_name = f"{base_name}{extension}"
            else:
                file_name = f"{base_name}_{counter}{extension}"

            if not os.path.exists(file_name):
                self.file_path = file_name
                break

            counter += 1

        if not os.path.isfile(self.file_path) or os.path.getsize(self.file_path) == 0:
            with open(self.file_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['time', 'hip_cont_angle','encoder_hip_angle','csv_hip_angle'])

    
        self.file = open(self.file_path, 'a', newline='')
        self.writer = csv.writer(self.file)

    
        self.start_time = time.time()

        self.timer = self.create_timer(0.03, self.log_data)

 
    def cont_callback(self, msg):
        self.hip_cont = msg.data[0]
        self.csv_angle = msg.data[1]

    def const_callback(self, msg):
        self.hip_const = msg.data
    def enc_callback(self,msg):
        self.enc_angle=msg.data
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