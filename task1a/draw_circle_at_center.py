import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, SetPen
import time

class DrawCircleAtCenter(Node):
    def __init__(self):
        super().__init__('draw_circle_at_center')

        # Publisher for velocity commands
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Create a client for the teleport service
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')

        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for teleport service...')

        # Parameters for circle
        self.circle_radius = 1.0  # Radius of the circle
        self.linear_speed = 1.0  # Speed of the turtle
        self.angular_speed = self.linear_speed / self.circle_radius  # Calculate angular speed

        # Draw the first circle at center (2.0, 2.0)
        self.draw_circle_at_center(2.0, 2.0)

        # Draw other circles
        self.draw_circle_at_center(8.0, 8.0)
        self.draw_circle_at_center(2.0, 8.0)
        self.draw_circle_at_center(8.0, 2.0)
    
    def set_pen(self, r, g, b, width, off):
        """Set the pen to the desired color and state (on/off)."""
        pen_req = SetPen.Request()
        pen_req.r = r  # Red component
        pen_req.g = g  # Green component
        pen_req.b = b  # Blue component
        pen_req.width = width  # Pen width
        pen_req.off = off  # 1 to turn the pen off, 0 to turn it on

        future = self.pen_client.call_async(pen_req)
        rclpy.spin_until_future_complete(self, future)

    def move_to_center(self, x, y):
        self.set_pen(0, 0, 0, 3, 1)
        """Teleport the turtle to the desired center (x, y)."""
        teleport_req = TeleportAbsolute.Request()
        teleport_req.x = x
        teleport_req.y = y
        teleport_req.theta = 0.0  # Facing forward
       
        future = self.teleport_client.call_async(teleport_req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Teleported to ({x}, {y})')
        else:
            self.get_logger().error('Teleport failed')

        time.sleep(1)  # Allow time for the turtle to reach the new position

    def draw_circle(self):
        self.set_pen(0, 0, 0, 3, 0)
        """Draw a circle at the current turtle position."""
        cmd = Twist()
        duration = (2 * 3.14159 * self.circle_radius) / self.linear_speed  # Time to complete the circle
        start_time = self.get_clock().now().to_msg().sec

        # Move the turtle in a circle for the calculated duration
        while self.get_clock().now().to_msg().sec - start_time < duration:
            cmd.linear.x = self.linear_speed
            cmd.angular.z = self.angular_speed
            self.publisher.publish(cmd)

        # Stop the turtle after drawing the circle
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.publisher.publish(cmd)

    def draw_circle_at_center(self, x, y):
        """Move to the specified center and draw a circle."""
        self.move_to_center(x, y)  # Move to the center
        self.draw_circle()  # Draw the circle

def main(args=None):
    rclpy.init(args=args)
    node = DrawCircleAtCenter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
