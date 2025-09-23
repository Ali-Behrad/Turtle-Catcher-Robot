import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill
from my_robot_interfaces.msg import TurtleArray, Turtle
from my_robot_interfaces.srv import DeleteTurtle
from random import uniform

class TurtleSpawnerNode(Node):
    def __init__(self):
        super().__init__(node_name="turtle_spawner_node")

        self.declare_parameter("spawn_frequency", value=1.0)
        
        self.x_y_theta = (0.0, 0.0, 0.0)
        self.alive_turtles_list = []

        self.client = self.create_client(srv_type=Spawn, srv_name="spawn")
        self.alive_turtles_publisher = self.create_publisher(msg_type=TurtleArray, topic="/alive_turtles", qos_profile=10)

        self.delete_turtle_server = self.create_service(srv_type=DeleteTurtle, srv_name="/delete_turtle", callback=self.delete_turtle_cb)
        self.kill_turtle_client = self.create_client(srv_type=Kill, srv_name="/kill")
        
        self.timer = self.create_timer(timer_period_sec=self.get_parameter("spawn_frequency").value, callback=self.turtle_spawner_cb)
    
    def publish_alive_turtles(self):
        msg = TurtleArray()
        msg.alive_turtles = self.alive_turtles_list

        self.alive_turtles_publisher.publish(msg=msg)

    def delete_turtle_cb(self, request: DeleteTurtle.Request, response: DeleteTurtle.Response):
        response.success = "ok"

        killed_turtle_name = request.name

        req = Kill.Request()
        req.name = killed_turtle_name

        self.get_logger().info("Killing the turtle")
        
        self.kill_turtle_client.call_async(request=req)

        idx = -1
        for (i, turtle) in enumerate(self.alive_turtles_list):
            if str(turtle.name) == str(killed_turtle_name):
                idx = i
                break

        self.alive_turtles_list.pop(idx)

        self.get_logger().info(f"The caught turtle name is {killed_turtle_name}")
        
        self.publish_alive_turtles()

        return response # Just do not FORGET!!!!!

    def turtle_spawner_cb(self):
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Spawn service is not up! Trying Again...")
        
        self.x_y_theta = (uniform(a=0.0, b=10.0), uniform(a=0.0, b=10.0), uniform(a=0.0, b=180.0))

        request = Spawn.Request()
        request.x = self.x_y_theta[0]
        request.y = self.x_y_theta[1]
        request.theta = self.x_y_theta[2]

        future = self.client.call_async(request)

        future.add_done_callback(self.handle_future_cb)
    
    def handle_future_cb(self, future):
        result = future.result()

        self.get_logger().info(f"[Spawned] {result.name} spawned at ({self.x_y_theta[0]}, {self.x_y_theta[1]}, {self.x_y_theta[2]})")
        
        turtle = Turtle()
        turtle.x = self.x_y_theta[0]
        turtle.y = self.x_y_theta[1]
        turtle.name = str(result.name)
        self.alive_turtles_list.append(turtle)

        # msg = TurtleArray()
        # msg.alive_turtles = self.alive_turtles_list

        # self.alive_turtles_publisher.publish(msg=msg)

        self.publish_alive_turtles()

def main():
    rclpy.init()
    node = TurtleSpawnerNode()

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()