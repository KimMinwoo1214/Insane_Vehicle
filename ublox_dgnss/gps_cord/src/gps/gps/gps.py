import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from ublox_ubx_msgs.msg import UBXNavHPPosLLH, UBXNavPVT
from std_msgs.msg import String
from geodesy.utm import fromLatLong


class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_cord_publisher')

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.gps_publisher = self.create_publisher(String, 'gps', 10)

        self.create_subscription(UBXNavHPPosLLH, '/ubx_nav_hp_pos_llh', self.hp_pos_callback, qos)
        self.create_subscription(UBXNavPVT, '/ubx_nav_pvt', self.pvt_callback, qos)

        self.latest_hp_pos = None
        self.latest_pvt = None

    def hp_pos_callback(self, msg):
        self.latest_hp_pos = msg
        self.publish_if_ready()

    def pvt_callback(self, msg):
        self.latest_pvt = msg
        self.publish_if_ready()

    def publish_if_ready(self):
        if self.latest_hp_pos and self.latest_pvt:
            utm_pt = fromLatLong(
                self.latest_hp_pos.lat / 10000000,
                self.latest_hp_pos.lon / 10000000
            )
            pub_list = [
                utm_pt.easting,
                utm_pt.northing,
                self.latest_pvt.head_mot / 100000,
                self.latest_hp_pos.lat / 10000000,
                self.latest_hp_pos.lon / 10000000
            ]
            msg_str = ','.join(map(str, pub_list))
            self.gps_publisher.publish(String(data=msg_str))
            self.get_logger().info(msg_str)


def main():
    rclpy.init()
    node = GPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
