#! /usr/bin/env python3

import socket
import re

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class UDPJoyNode(Node):

    def __init__(self):
        super().__init__('udpjoy_node')

        self.declare_parameter("port", 37501)

        port = self.get_parameter("port").value
        self.server_addr = ('', port)

        self.MAX_SIZE = 1024

        self.sock = socket.socket(socket.AF_INET, type=socket.SOCK_DGRAM)
        self.sock.bind(self.server_addr)

        self.parser = re.compile(r'([-?\d.]+)')

        self.joy_pub = self.create_publisher(Joy, 'udp_joy/data', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def __del__(self):
        self.sock.close()


    def timer_callback(self):
        rx_meesage, addr = self.sock.recvfrom(self.MAX_SIZE)
        msg = Joy()
        raw_data = rx_meesage.decode(encoding='utf-8')
        if(len(rx_meesage) > 0):
            result = self.parser.findall(raw_data)
            if len(result) > 3:
                msg.axes.append(float(result[2]))
                msg.axes.append(float(result[3]))
                msg.axes.append(float(result[0]))
                msg.axes.append(float(result[1]))

                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'joy'

                self.joy_pub.publish(msg)

def main():
    rclpy.init()

    node = UDPJoyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
