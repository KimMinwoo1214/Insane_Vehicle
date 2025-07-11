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
import utm

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
        
        self.targets = [
            {"lat_lon": (33.305571, 126.313477), "action": {"angle": 120.0, "duration": 4.0}},
            {"lat_lon": (36.032394, 128.203948), "action": {"angle": 120.0, "duration": 0.5}}
        ]
        self.target_index = 0
        self.current_target_utm = None
        self.action_triggered_for_target = False
        self.threshold = 0.05  # 5 centimeters

        self.update_current_target()

    def update_current_target(self):
        if self.target_index < len(self.targets):
            lat, lon = self.targets[self.target_index]["lat_lon"]
            self.current_target_utm = utm.from_latlon(lat, lon)[:2]
            self.action_triggered_for_target = False
            self.get_logger().info(f"Set new target: {self.targets[self.target_index]['lat_lon']} (UTM: {self.current_target_utm})")
        else:
            self.current_target_utm = None
            self.get_logger().info("All targets reached. Node will continue to monitor GPS but no further actions.")

    def gps_callback(self, msg):
        self.current_pos = (msg.x, msg.y)
        self.check_distance_and_act()

    def check_distance_and_act(self):
        # Proceed only if we have a position and the action hasn't been triggered yet for the current target
        if self.current_pos and self.current_target_utm and not self.action_triggered_for_target:
            distance = math.sqrt((self.current_pos[0] - self.current_target_utm[0])**2 + (self.current_pos[1] - self.current_target_utm[1])**2)

            if distance < self.threshold:
                self.execute_action(self.targets[self.target_index]["action"])
                self.action_triggered_for_target = True # Mark action as done for this target
                self.get_logger().info(f"Action triggered for target {self.targets[self.target_index]['lat_lon']} and will not be triggered again.")
                # Move to the next target after action is triggered
                self.target_index += 1
                self.update_current_target()


    def execute_action(self, action_params):
        angle = action_params["angle"]
        duration = action_params["duration"]
        self.get_logger().info(f"Target {self.current_target_utm} reached. Publishing {angle} degrees for {duration} seconds.")

        # Use a timer for non-blocking publishing
        self.action_start_time = self.get_clock().now()
        self.action_duration = duration # Store duration for publish_action_message
        self.action_angle = angle # Store angle for publish_action_message
        self.timer = self.create_timer(0.1, self.publish_action_message) # Publish at 10 Hz

    def publish_action_message(self):
        duration_sec = (self.get_clock().now() - self.action_start_time).nanoseconds / 1e9

        if duration_sec < self.action_duration:
            msg = Float32()
            msg.data = self.action_angle
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
