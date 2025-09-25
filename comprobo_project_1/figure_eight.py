"""
figure_eight Node
----------------
This node tells the robot to draw a figure eight pattern.
"""

import rclpy
from rclpy.node import Node
from threading import Thread, Event
from time import sleep
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import math

class FigureEight(Node):
    """
    A node that tells the robot to draw a figure eight pattern.
    """

    def __init__(self):
        super().__init__('figure_eight')
        self.e_stop = Event()
        # create a thread to handle long-running component
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Bool, 'estop', self.handle_estop, 10)
        self.run_loop_thread = Thread(target=self.run_loop)
        self.run_loop_thread.start()


    def handle_estop(self, msg):
        """
        Handles messages received on the estop topic.

        Args:
            msg (std_msgs.msg.Bool): the message that takes value true if we
            estop and false otherwise.
        """ 
        if msg.data:
            self.e_stop.set()
            self.drive(linear=0.0, angular=0.0)

    def run_loop(self):
        """
        Executes the main logic for driving the figure eight.  This function
        does not return until the figure eight is finished or the estop is
        pressed.
        """
        # the first message on the publisher is often missed
        self.drive(0.0, 0.0)
        sleep(1)
        print('starting run loop')

        linear_vel = 0.1
        angular_vel = 0.3
        delta_ang = 0.01
        self.drive(linear=linear_vel, angular=angular_vel)
        sleep(math.pi / abs(angular_vel))
        while angular_vel > -0.3:
            angular_vel -= delta_ang
            self.drive(linear=linear_vel, angular=angular_vel)
            sleep(20 * delta_ang)
        sleep(math.pi / abs(angular_vel))
        while angular_vel < 0.3:
            angular_vel += delta_ang
            self.drive(linear=linear_vel, angular=angular_vel)
            sleep(20 * delta_ang)
        self.drive(linear=0.0, angular=0.0)

        print('done with run loop')
        self.destroy_node()
        rclpy.shutdown()

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

def main():
    rclpy.init()
    figure_eight = FigureEight()
    rclpy.spin(figure_eight)


if __name__ == '__main__':
    main()