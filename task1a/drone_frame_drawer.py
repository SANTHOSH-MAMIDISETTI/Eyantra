import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute, SetPen
from geometry_msgs.msg import Twist

class DroneFrameDrawer(Node):
    def __init__(self):
        super().__init__('drone_frame_drawer')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # Set up the service clients for teleporting and pen control
        self.client_teleport = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.client_pen = self.create_client(SetPen, '/turtle1/set_pen')
        
        # Wait for services to be available
        while not self.client_teleport.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for teleport service...')
        while not self.client_pen.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_pen service...')
        
        # Initial teleport and pen settings
        self.teleport_turtle(3.0, 5.0, -0.785)  # Start with 45-degree orientation
        self.set_pen_state(False)  # Set pen off
        self.timer = self.create_timer(0.1, self.control_turtle)
        self.state = 'move'
        self.count = 0
        self.start_time = self.get_clock().now()

    def set_pen_state(self, on):
        """Enable or disable the pen."""
        req = SetPen.Request()
        req.r = 255
        req.g = 255
        req.b = 255
        req.width = 2
        req.off = 0 if on else 1
        self.client_pen.call_async(req)

    def teleport_turtle(self, x, y, theta):
        """Teleport the turtle to a specified position."""
        req = TeleportAbsolute.Request()
        req.x = x
        req.y = y
        req.theta = theta
        self.client_teleport.call_async(req)

    def control_turtle(self):
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9  # Convert to seconds

        move_cmd = Twist()
        turn_cmd = Twist()
        self.set_pen_state(True)

        # Fine-tuned state machine for accurate movement and turning
        if self.state == 'move' and elapsed_time >= 2.0:  # Move for 2 seconds
            self.state = 'stop_move'
            self.start_time = current_time
        elif self.state == 'stop_move' and elapsed_time >= 0.2:  # Short stop before turning
            self.state = 'turn'
            self.start_time = current_time
        elif self.state == 'turn' and elapsed_time >= 0.9:  # Turn to make the diagonal square pattern
            self.state = 'stop_turn'
            self.start_time = current_time
        elif self.state == 'stop_turn' and elapsed_time >= 0.2:  # Short stop after turning
            self.state = 'move'
            self.start_time = current_time
            self.count += 1

        # Execute commands based on the state
        if self.state == 'move':
            move_cmd.linear.x = 1.0  # Move forward at 1 m/s
            self.publisher_.publish(move_cmd)
            self.get_logger().info('Moving forward...')
        elif self.state == 'turn':
            turn_cmd.angular.z = 1.57  # Turn at 1.57 rad/s
            self.publisher_.publish(turn_cmd)
            self.get_logger().info('Turning 90 degrees...')
        else:
            # Publish a zero command to stop the turtle
            self.publisher_.publish(Twist())

        # Stop after completing four sides of the square
        if self.count >= 4:
            self.publisher_.publish(Twist())  # Send a zero Twist to stop the turtle
            self.get_logger().info('Square completed!')
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    drone_frame_drawer = DroneFrameDrawer()
    rclpy.spin(drone_frame_drawer)
    drone_frame_drawer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
