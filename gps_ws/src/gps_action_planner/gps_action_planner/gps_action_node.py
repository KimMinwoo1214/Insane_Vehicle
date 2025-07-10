#1. Source your workspace:
# source /home/m2nyok2/PycharmProjects/Insane_Vehicle/gps_ws/ublox_dgnss/install/setup.zsh
#2. Update the target coordinate: Open the file /home/m2nyok2/PycharmProjects/Insane_Vehicle/gps_ws/src/gps_action_planner/gps_action_planner/gps_action_node.py and change the self.target_pos variable to your precise UTM coordinate.
#3. Run the node:
# ros2 run gps_action_planner gps_action_node



import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
import math
import time

class GpsActionNode(Node):
    def __init__(self):
        super().__init__('gps_action_node')
        self.subscription = self.create_subscription(
            Point,
            '/gps',
            self.gps_callback,
            10)
        self.publisher = self.create_publisher(Float32, 'gps_steering_angle', 10)
        self.current_pos = None
        self.action_in_progress = False
        self.action_start_time = None
        self.action_triggered_for_target = False

        # --- Hard-coded target GPS coordinate ---
        # Please replace this with your actual UTM coordinate.
        self.target_pos = (302333.0, 4122444.0)
        # -----------------------------------------

        self.threshold = 0.05  # 5 centimeters

    def gps_callback(self, msg):
        self.current_pos = (msg.x, msg.y)
        self.check_distance_and_act()

    def check_distance_and_act(self):
        # Proceed only if we have a position and the action hasn't been triggered yet
        if self.current_pos and not self.action_triggered_for_target:
            distance = math.sqrt((self.current_pos[0] - self.target_pos[0])**2 + (self.current_pos[1] - self.target_pos[1])**2)
            
            if distance < self.threshold:
                self.execute_action()
                self.action_triggered_for_target = True # Mark action as done for this target
                self.get_logger().info(f"Action triggered for target {self.target_pos} and will not be triggered again.")


    def execute_action(self):
        self.get_logger().info(f"Target {self.target_pos} reached. Publishing 120 degrees for 4 seconds.")
        
        # Use a timer for non-blocking publishing
        self.action_start_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.publish_action_message) # Publish at 10 Hz

    def publish_action_message(self):
        duration_sec = (self.get_clock().now() - self.action_start_time).nanoseconds / 1e9
        
        if duration_sec < 4.0:
            msg = Float32()
            msg.data = 120.0
            self.publisher.publish(msg)
        else:
            self.get_logger().info('Action finished.')
            self.timer.cancel() # Stop the timer


def main(args=None):
    rclpy.init(args=args)
    gps_action_node = GpsActionNode()
    rclpy.spin(gps_action_node)
    gps_action_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
