import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry

from tf_transformations import euler_from_quaternion

from time import sleep
from math import sqrt, cos, sin


class Mission(Node):
    def __init__(self):
        Node.__init__(self, node_name="nav")

        self.x, self.y, self.z, self.yaw = None, None, None, None  # Current pose: x, y, z, yaw
        self.goal_reached = False
        
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 1)   #Publisher used to send the velocity commands to the drone via the /drone1/cmd_vel topic
        self.take_off_pub = self.create_publisher(Empty, "takeoff", 1)  #Publisher used to send a message on the /drone1/takeoff topic
        self.land_pub = self.create_publisher(Empty, "land", 1) #Publisher used to send a message on the /drone1/land topic
        self.odom_sub = self.create_subscription(Odometry, "odom", self.odom_cb, 10)    #Subscriber used to get the drone's pose over time
        
        self.get_params()
        self.take_off()
        self.create_timer(0.1, self.run)  # Call run() every 0.1 seconds
    

    
    def get_params(self):
        """
        Description
            ------------
                Get the target pose the UAV has to reach.

            Returns
            -------
                None
        """
        self.declare_parameters(
            namespace="",
            parameters=[
                ("x_goal", rclpy.Parameter.Type.DOUBLE),    #robot's diameter in meter
                ("y_goal", rclpy.Parameter.Type.DOUBLE),
                ("z_goal", rclpy.Parameter.Type.DOUBLE),
                ("yaw_goal", rclpy.Parameter.Type.DOUBLE),
            ]
        )
        self.target_x = self.get_parameter("x_goal").value
        self.target_y = self.get_parameter("y_goal").value
        self.target_z = self.get_parameter("z_goal").value
        self.target_yaw = self.get_parameter("yaw_goal").value
        self.get_logger().info(f"Target pose: ({self.target_x:.2f}, {self.target_y:.2f}, {self.target_z:.2f}, {self.target_yaw:.2f})")
    

    def take_off(self):
        """
        Description
            ------------
                Publish a message on the takeoff topic of the drone's controller.

            Returns
            -------
                None
        """
        self.take_off_pub.publish(Empty())

    
    def land(self):
        """
        Description
            ------------
                Publish a message on the land topic of the drone's controller.

            Returns
            -------
                None
        """
        self.land_pub.publish(Empty())


    def odom_cb(self, msg):
        """
        Description
        ------------
            Get the UAV's position and heading. 
            This method is automatically called whenever a new message is published on the topic /drone1/odom.
        
        Parameters
        ----------
            msg : Odometry
                Message received on the topic /drone1/odom.
        
        Returns
        -------
            None
        """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z
        self.yaw = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]


    def run(self):
        if self.x is not None:
            cmd_vel = Twist()

            if not self.goal_reached:
                self.get_logger().info(f"Current pose: ({self.x:.2f}, {self.y:.2f}, {self.z:.2f}, {self.yaw:.2f})")
                
                # TODO: implement the strategy here
                """
                cmd_vel.linear.x = ... (it corresponds to u_d)
                cmd_vel.linear.y = ... (it corresponds to v_d)
                cmd_vel.linear.z = ... (it corresponds to w_d)
                cmd_vel.angular.z = ... (it corresponds to r_d)
                """

            self.cmd_vel_pub.publish(cmd_vel)


    def distance(self, p1, p2):
        return sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2 + (p1[2] - p2[2]) ** 2)


def main():
    rclpy.init()

    node = Mission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()