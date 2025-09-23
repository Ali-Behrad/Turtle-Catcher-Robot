import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import DeleteTurtle
from math import sqrt, atan2, pi
import time
import numpy as np

class TurtleMoverNode(Node):
    def __init__(self):
        super().__init__(node_name="turtle_mover")

        self.declare_parameter("starting_pose", value=[5.5, 5.5, 0.0, 0.0, 0.0])

        self.dest = None
        self.turtle_pose = self.get_parameter("starting_pose").value

        self.alive_turtles = []
        self.idx = -1

        self.target_lock = 0
        
        self.dist, self.steer= 0.0, 0.0

        self.publisher = self.create_publisher(msg_type=Twist, topic="/turtle1/cmd_vel", qos_profile=10)
        self.alive_turtles_subscriber = self.create_subscription(msg_type=TurtleArray, topic="/alive_turtles", callback=self.get_alive_turtles_cb, qos_profile=10)
        
        self.pose_subscriber = self.create_subscription(msg_type=Pose, topic="/turtle1/pose", callback=self.get_pose_cb, qos_profile=10)

        self.kill_turtle_client = self.create_client(srv_type=DeleteTurtle, srv_name="/delete_turtle")

    def get_alive_turtles_cb(self, msg: TurtleArray):
        if len(msg.alive_turtles) > 0 and not self.target_lock:
            self.alive_turtles = msg.alive_turtles

            positions = np.array([
                [t.x, t.y] for t in msg.alive_turtles 
            ])

            master = np.array(self.turtle_pose[:2])

            dists = np.linalg.norm(positions - master, axis=1)

            idx = np.argmin(dists)

            self.idx = idx

            self.dest = (msg.alive_turtles[idx].x, msg.alive_turtles[idx].y)
        

    def get_pose_cb(self, msg: Pose):
        self.turtle_pose = (msg.x, msg.y, msg.theta, msg.linear_velocity, msg.angular_velocity)

        self.movement_policy(self.turtle_pose)

    def movement_policy(self, turtle_pose: tuple):
        if self.dest == None:
            return
        
        velocities = Twist()

        self.target_lock = 1
        
        Kp = 2
        
        self.dist = sqrt((turtle_pose[0] - self.dest[0]) ** 2 + (turtle_pose[1] - self.dest[1]) ** 2)

        self.steer = atan2((self.dest[1] - turtle_pose[1]), (self.dest[0] - turtle_pose[0])) - turtle_pose[2]

        if self.steer > pi:
            self.steer -= 2 * pi
        elif self.steer < -pi:
            self.steer += 2 * pi 

        velocities.linear.x = Kp * self.dist 
        velocities.linear.y = 0.3 * Kp * self.dist  # drift on the y axis
        velocities.angular.z =  3 * Kp * self.steer

        if self.dist < 0.5:
            self.get_logger().info("Reached the Destination")
            velocities.linear.x = 0.0
            velocities.angular.z = 0.0

            if len(self.alive_turtles) > 0:
                req = DeleteTurtle.Request()
                req.name = self.alive_turtles[self.idx].name

                self.get_logger().warn("Going to kill the turtle")
                self.kill_turtle_client.call_async(req)
                self.dest = None
                self.target_lock = 0

        self.publisher.publish(velocities)

        time.sleep(0.1)

def main():
    rclpy.init()
    node = TurtleMoverNode()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()