"""
teleop Node
--------
This node allows for teleoperation of a robot using keyboard inputs.
"""

import tty
import select
import sys
import termios

import rclpy
from rclpy.node import Node
from threading import Thread, Event
from time import sleep
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import math

class TeleOp(Node):
    """
    A node that allows for teleoperation of a robot using keyboard inputs.
    """

    def __init__(self):
        super().__init__('teleop')
        self.e_stop = Event()
        # create a thread to handle long-running component
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Bool, 'estop', self.handle_estop, 10)
        self.run_loop_thread = Thread(target=self.run_loop)
        self.run_loop_thread.start()


    def handle_estop(self, msg):
        """Handles messages received on the estop topic.

        Args:
            msg (std_msgs.msg.Bool): the message that takes value true if we
            estop and false otherwise.
        """ 
        if msg.data:
            self.e_stop.set()
            self.drive(linear=0.0, angular=0.0)

    def run_loop(self):
        """
        Execute the main loop for teleoperation.  This function does not return until
        the user exits the program by pressing Ctrl-C.
        """
        # the first message on the publisher is often missed
        self.drive(0.0, 0.0)
        sleep(1)

        current_facing = 0  # 0 = north, 1 = east, 2 = south, 3 = west
        key = None
        while key != '\x03':
            key = self.getKey()
            print(key)

            if key == 'w':
                self.turn_and_drive(current_facing, 0)
                current_facing = 0
            elif key == 'd':
                self.turn_and_drive(current_facing, 1)
                current_facing = 1
            elif key == 's':
                self.turn_and_drive(current_facing, 2)
                current_facing = 2
            elif key == 'a':
                self.turn_and_drive(current_facing, 3)
                current_facing = 3
            elif key == ' ':
                self.drive(linear=0.0, angular=0.0)

        self.handle_estop(Bool(data=True))
        print("Exiting teleop")
        self.destroy_node()
        rclpy.shutdown()
    
    def turn_and_drive(self, current_facing, target_facing):
        """
        Spin to face the target direction and drive forward.

        Args:
            current_facing (int): the current facing direction
            target_facing (int): the target facing direction
        """
        if current_facing == target_facing:
            self.drive_forward()
        elif (current_facing + 1) % 4 == target_facing:
            self.turn_right()
            self.drive_forward()
        elif (current_facing + 3) % 4 == target_facing:
            self.turn_left()
            self.drive_forward()
        elif (current_facing + 2) % 4 == target_facing:
            self.turn_left()
            self.turn_left()
            self.drive_forward()

    def drive(self, linear, angular):
        """
        Drive with the specified linear and angular velocity.

        Args:
            linear (_type_): the linear velocity in m/s
            angular (_type_): the angular velocity in radians/s
        """        
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.vel_pub.publish(msg)

    def turn_left(self):
        """
        Execute a 90 degree left turn
        """
        angular_vel = 0.3
        if not self.e_stop.is_set():
            self.drive(linear=0.0, angular=angular_vel)
            sleep(math.pi / angular_vel / 2)
    
    def turn_right(self):
        """
        Execute a 90 degree right turn
        """
        angular_vel = -0.3
        if not self.e_stop.is_set():
            self.drive(linear=0.0, angular=angular_vel)
            sleep(math.pi / abs(angular_vel) / 2)

    def drive_forward(self):
        """
        Drive straight for the spefcified distance.
        """
        forward_vel = 0.1
        if not self.e_stop.is_set():
            self.drive(linear=forward_vel, angular=0.0)

    def getKey(self):
        """
        Get a single keypress from the keyboard.
        """
        fd = sys.stdin.fileno()
        # save original terminal settings so we can restore them
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            select.select([sys.stdin], [], [], 0)
            key = sys.stdin.read(1)
        finally:
            # restore original terminal settings
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

def main():
    rclpy.init()
    teleop = TeleOp()
    rclpy.spin(teleop)


if __name__ == '__main__':
    main()
