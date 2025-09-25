import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import qos_profile_sensor_data

class WallApproachNode(Node):
    """ This class wraps the basic functionality of the node """
    def __init__(self):
        super().__init__('wall_approach')
        # the run_loop adjusts the robot's velocity based on latest laser data
        self.create_timer(0.1, self.run_loop)
        self.create_subscription(LaserScan, 'scan', self.process_scan, qos_profile=qos_profile_sensor_data)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # distance_to_walls are used to communciate laser data to run_loop
        self.distance_to_wall_45 = None
        self.distance_to_wall_90 = None
        self.distance_to_wall_135 = None

        # Kp is the constant or to apply to the proportional error signal
        self.Kp = 0.4
        # target_dist is the desired distance to the obstacle at the different angles
        self.target_dist_45 = 0.424267
        self.target_dist_90 = 0.3
        self.target_dist_135 = 0.424267
        self.target_dist_range = 0.3


    def run_loop(self):
        msg = Twist()
        #testing originally with just 45 degrees
        if self.distance_to_wall_45 > target_dist_45 + self.target_dist_range
        #adjust a certain number of degrees towards wall depending on how far out of range we are 
        if self.distance_to_wall_45 < target_dist_45 - self.target_dist_range
        #adjust a certain number of degrees away from wall depending on how far out of range we are 

        self.vel_pub.publish(msg)

    def process_scan(self, msg):
        if msg.ranges[0] != 0.0:
            # checking for the value 0.0 ensures the data is valid.
            # Your logic here!
            #get the distances with lidar data
            self.distance_to_wall_45 = 
            self.distance_to_wall_90 = 
            self.distance_to_wall_135 = 
            print('scan received', msg.ranges[0])


def main(args=None):
    rclpy.init(args=args)
    node = WallApproachNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()