import tty
import select
import sys
import termios

import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from neato2_interfaces.msg import Bump
from time import sleep, time
from threading import Thread
import math

class FiniteStateController(Node):
    def __init__(self):
        super().__init__('finite_state_controller')
        self.state = "teleop"
        self.changed_state = True
        self.default_linear_vel = 0.2
        self.default_angular_vel = 0.6
        self.bumped = False

        self.create_timer(0.1, self.run_loop)
        
        # creates publisher for cmd_vel
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        # creates subscribers for lidar and bump
        self.scan_subscriber = self.create_subscription(
            LaserScan, "scan", self.process_scan, qos_profile=qos_profile_sensor_data
        )
        self.bump_subscriber = self.create_subscription(
            Bump, "bump", self.handle_bump, 10
        )

        # Variables for teleop
        self.teleop_thread = Thread(target=self.teleop)
        self.current_facing = 0
        self.right_start = None
        self.left_start = None
        # Variables for wall follow
        self.wall_follow_thread = Thread(target=self.wall_follow)
        self.linear_vel = self.default_linear_vel
        self.angular_vel = 0.0
        self.direction = 'n'  # 'n' for no turn, 'l' for left, 'r' for right
        self.distance_to_obstacle = None
        self.distances = None
        # Variables for bump reaction
        self.dizzy_thread = Thread(target=self.dizzy_reaction)

        
    
    def run_loop(self):
        """
        The main loop of the finite state controller. This function is called
        periodically by a timer.
        """
        if self.changed_state:
            self.changed_state = False
            self.drive(0.0, 0.0)
            sleep(0.1)

            # start the appropriate thread based on the current state
            if self.state == "teleop":
                self.teleop_thread = Thread(target=self.teleop)
                self.teleop_thread.start()
            elif self.state == "wall_follow":
                self.wall_follow_thread = Thread(target=self.wall_follow)
                self.wall_follow_thread.start()
            elif self.state == "shutdown":
                print("Shutting down.")
                self.drive(0.0, 0.0)
                self.destroy_node()
                rclpy.shutdown()
                return
            
        if self.bumped:
            self.bumped = False
            print("Bumped! Exiting current state.")
            # stop the current thread
            if self.state == "teleop":
                self.state = "wall_follow"
                self.teleop_thread.join()
            elif self.state == "wall_follow":
                self.state = "teleop"
                self.wall_follow_thread.join()
            self.changed_state = True
            # perform the dizzy reaction
            self.dizzy_thread = Thread(target=self.dizzy_reaction)
            self.dizzy_thread.start()
            self.dizzy_thread.join()
            
    
    def teleop(self):
        """
        Execute the main loop for teleoperation.  This function does not return until
        the user exits the program by pressing Ctrl-C or the bump sensor is activated.
        """
        print("Starting teleop.  Use WASD or IJKL to drive, space to stop, and Ctrl-C to quit.")
        
        key = self.getKey()
        while self.state == "teleop":
            key = self.getKey()
            if key is None:
                continue
            print(key)
            if key in ['w', 'a', 's', 'd', ' ', 'i', 'k', 'j', 'l', '8']:
                # Determine current facing before changing state
                if self.right_start is not None:
                    self.drive(0.0, 0.0)
                    self.current_facing = (self.current_facing + math.degrees(self.default_angular_vel) * (time() - self.right_start)) % 360
                    print(self.current_facing)
                    self.right_start = None
                if self.left_start is not None:
                    self.drive(0.0, 0.0)
                    self.current_facing = (self.current_facing - math.degrees(self.default_angular_vel) * (time() - self.left_start)) % 360
                    print(self.current_facing)
                    self.left_start = None
                # Key handling
                if key == 'w':
                    self.turn_and_drive(self.current_facing, 0, self.default_linear_vel, self.default_angular_vel)
                    self.current_facing = 0
                elif key == 'd':
                    self.turn_and_drive(self.current_facing, 90, self.default_linear_vel, self.default_angular_vel)
                    self.current_facing = 90
                elif key == 's':
                    self.turn_and_drive(self.current_facing, 180, self.default_linear_vel, self.default_angular_vel)
                    self.current_facing = 180
                elif key == 'a':
                    self.turn_and_drive(self.current_facing, 270, self.default_linear_vel, self.default_angular_vel)
                    self.current_facing = 270
                elif key == ' ':
                    self.drive(0.0, 0.0)
                elif key == 'i':  # up arrow
                    self.drive(self.default_linear_vel, 0.0)
                elif key == 'k':  # down arrow
                    self.drive(-self.default_linear_vel, 0.0)
                elif key == 'j':  # left arrow
                    self.drive(0.0, self.default_angular_vel)
                    self.left_start = time()
                elif key == 'l':  # right arrow
                    self.drive(0.0, -self.default_angular_vel)
                    self.right_start = time()
                elif key == '8':  # figure eight
                    self.figure_eight()
            elif key == '\x03':  # Ctrl-C
                self.state = "shutdown"
                self.changed_state = True
        self.drive(0.0, 0.0)
            
    def wall_follow(self):
        """
        Execute the main loop for wall following. This function does not return until
        the bump sensor is activated.
        """

        print("Starting wall follow.")
        target_distance = 1.0  # desired distance from wall in meters
        Kp = 0.5  # proportional control constant
        buffer = 0.1

        while self.state == "wall_follow":
            if self.direction == 'n':
                if self.distance_to_obstacle is None:
                    # if we haven't seen an obstacle yet, just go straight at fixed vel
                    self.linear_vel = self.default_linear_vel
                    self.angular_vel = 0.0
                elif self.distance_to_obstacle > target_distance:
                    # use proportional control to set the velocity
                    self.linear_vel = min(Kp*(self.distance_to_obstacle - target_distance + buffer), self.default_linear_vel)
                    self.angular_vel = 0.0
                else:
                    if self.distances[45] < self.distances[315]:
                        #turn left
                        self.angular_vel = self.default_angular_vel
                        self.direction = 'l'          
                    else:
                        #turn right
                        self.angular_vel = -self.default_angular_vel
                        self.direction = 'r'
            elif self.direction == 'l':
                self.linear_vel = min(Kp*(self.distance_to_obstacle - target_distance + buffer), self.default_linear_vel)
                if min(self.distances[0:61]) <= target_distance:
                    self.angular_vel = -self.default_angular_vel
                elif min(self.distances[0:61]) > target_distance:
                    self.angular_vel = self.default_angular_vel
            elif self.direction == 'r':
                self.linear_vel = min(Kp*(self.distance_to_obstacle - target_distance + buffer), self.default_linear_vel)
                if min(self.distances[300:361]) <= target_distance:
                    self.angular_vel = -self.default_angular_vel
                elif min(self.distances[300:361]) > target_distance:
                    self.angular_vel = self.default_angular_vel
            self.drive(self.linear_vel, self.angular_vel)
        self.drive(0.0, 0.0)

    def dizzy_reaction(self):
        """
        Execute the dizzy reaction. This function does not return until
        the reaction is finished.
        """
        print("starting dizzy reaction")
        linear_vel = -0.2
        angular_vel = 0.3
        delta_ang = 0.1
        while angular_vel > -0.3:
            self.drive(linear=linear_vel, angular=angular_vel)
            sleep(30 * delta_ang * abs(angular_vel))
            angular_vel -= delta_ang
        while angular_vel < 0.3:
            self.drive(linear=linear_vel, angular=angular_vel)
            sleep(30 * delta_ang * abs(angular_vel))
            angular_vel += delta_ang
        self.drive(linear=0.0, angular=0.0)

    def figure_eight(self):
        """
        Execute a figure eight maneuver. This function does not return until
        the maneuver is finished.
        """
        print("starting figure-eight")

        linear_vel = 0.1
        angular_vel = 0.3
        delta_ang = 0.01
        while angular_vel > -0.3:
            angular_vel -= delta_ang
            self.drive(linear=linear_vel, angular=angular_vel)
            sleep(20 * delta_ang)
        sleep(math.pi / abs(angular_vel))
        while angular_vel < 0.3:
            angular_vel += delta_ang
            self.drive(linear=linear_vel, angular=angular_vel)
            sleep(20 * delta_ang)
        sleep(math.pi / abs(angular_vel))
        self.drive(linear=0.0, angular=0.0)

    def process_scan(self, msg):
        """
        Processes the incoming LIDAR scan data.
        """
        if msg.ranges[0]!= 0.0:
            # checking for the value 0.0 ensures the data is valid.
            self.distance_to_obstacle = msg.ranges[0]
            self.distances = np.array(msg.ranges)

    def handle_bump(self, msg):
        """
        Handles the bump state of the neato
        """
        # Checks to see if the bump state is true from the neato
        self.bumped = (
            msg.left_front or msg.right_front or msg.left_side or msg.right_side
        )
    
    def drive(self, linear, angular):
        """
        Drive with the specified linear and angular velocity.

        Args:
            linear: the linear velocity in m/s
            angular: the angular velocity in radians/s
        """        
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.vel_pub.publish(msg)
    
    def turn_and_drive(self, current_facing, target_facing, linear_vel, angular_vel):
        """
        Spin to face the target direction and drive forward.
        Used by teleop function.

        Args:
            current_facing (int): the current facing direction
            target_facing (int): the target facing direction
        """
        delta_angle = (target_facing - current_facing) % 360
        if delta_angle <= 180:
            self.drive(0.0, -angular_vel)
            sleep(math.radians(delta_angle) / angular_vel)
        else:
            self.drive(0.0, angular_vel)
            sleep(math.radians(360 - delta_angle) / angular_vel)
        self.drive(linear_vel, 0.0)
    
    def getKey(self, timeout=0.1):
        """
        Get a single keypress from the keyboard.
        Used by teleop function.
        """
        fd = sys.stdin.fileno()
        # save original terminal settings so we can restore them
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ready, _, _ = select.select([sys.stdin], [], [], timeout)
            if ready:
                return sys.stdin.read(1)
            return None
        finally:
            # restore original terminal settings
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def main():
    rclpy.init()
    node = FiniteStateController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()