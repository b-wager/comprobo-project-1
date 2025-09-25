""" This node uses the laser scan measurement pointing straight ahead from
    the robot and compares it to a desired set distance.  The forward velocity
    of the robot is adjusted until the robot achieves the desired distance """

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import qos_profile_sensor_data
import numpy as np

class WallApproachNode(Node):
    """ This class wraps the basic functionality of the node """
    def __init__(self):
        super().__init__('wall_approach')
        # the run_loop adjusts the robot's velocity based on latest laser data
        self.create_timer(0.1, self.run_loop)
        self.create_subscription(LaserScan, 'scan', self.process_scan, qos_profile=qos_profile_sensor_data)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # distance_to_obstacle is used to communciate laser data to run_loop
        self.distance_to_obstacle = None
        # Kp is the constant or to apply to the proportional error signal
        self.Kp = 0.4
        # target_distance is the desired distance to the obstacle in front
        self.target_distance = 0.6
        self.buffer = 1.4
        self.distances = None
        self.angular_vel = 0
        self.turned = 'n'

    def run_loop(self):
        msg = Twist()
        if self.turned == 'n':
            if self.distance_to_obstacle is None:
                # if we haven't seen an obstacle yet, just go straight at fixed vel
                msg.linear.x = 0.1
                msg.angular.z = 0.0
            elif self.distance_to_obstacle > self.target_distance:
                # use proportional control to set the velocity
                msg.linear.x = self.Kp*(self.distance_to_obstacle - self.target_distance)
                if msg.linear.x > 0.3:
                    msg.linear.x = 0.3
                msg.angular.z = 0.0
            else:
                self.turn_at_wall(msg)
                msg.linear.x = 0.1
                msg.angular.z = self.angular_vel

        elif self.turned == 'l':
            msg.linear.x = self.Kp*(self.distance_to_obstacle - self.target_distance)
            if msg.linear.x > 0.3:
                msg.linear.x = 0.3
            if min(self.distances[0:61]) < self.target_distance:
                msg.angular.z = 0.3
            elif min(self.distances[0:61]) > self.target_distance:
                msg.angular.z = -0.3
            else:
                msg.angular.z = 0.0
                self.turned = 'n'

        elif self.turned == 'r':
            msg.linear.x = self.Kp*(self.distance_to_obstacle - self.target_distance)
            if msg.linear.x > 0.3:
                msg.linear.x = 0.3
            if min(self.distances[300:361]) < self.target_distance:
                msg.angular.z = 0.3
            elif min(self.distances[300:361]) > self.target_distance:
                msg.angular.z = -0.3
            else:
                msg.angular.z = 0.0
                self.turned = 'n'

        self.vel_pub.publish(msg)
    

    def process_scan(self, msg):
        if msg.ranges[0] != 0.0:
            # checking for the value 0.0 ensures the data is valid.
            self.distance_to_obstacle = msg.ranges[0]
            self.distances = np.array(msg.ranges)
    
    def turn_at_wall(self, msg):
        if self.distances[45] < self.distances[315]:
            #turn left
            print("left turn")
            self.angular_vel = 0.3
            self.turned = 'l'

            
        else:
            #turn right
            print("right turn")
            self.angular_vel = -0.3
            self.turned = 'r'
        
        
        




def main(args=None):
    rclpy.init(args=args)
    node = WallApproachNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()