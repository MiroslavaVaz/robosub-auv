
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA


class EnvironmentMarkers(Node):
    """
    Publishes a MarkerArray that draws:
      - Start cube
      - Three landmark spheres
      - Gate posts + crossbar
      - Three channel bars
    in the 'map' frame, on /robosub_environment.
    """

    def __init__(self):
        super().__init__('environment_markers')

        # Publisher (this is what RViz subscribes to)
        self.pub = self.create_publisher(
            MarkerArray, '/robosub_environment', 10
        )

        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Environment marker publisher started.')

        # ----- Layout (map frame, meters) -----
        # Robot start position (edge of pool)
        self.start = (-8.0, 0.0)

        # Landmarks the robot must detect (white spheres)
        self.landmarks = [
            (-2.0,  2.0),
            ( 2.0, -2.0),
            ( 6.0,  1.0),
        ]

        # Gate posts (PVC-style) – roughly like the entrance gate
        self.gate_posts = [
            (-9.0, -1.0),
            (-9.0,  1.0),
        ]

        # Channel bars (PVC bars to swim over/around)
        self.channel_bars = [
            (-4.0, 0.0),
            ( 0.0, 0.0),
            ( 4.0, 0.0),
        ]

        # ----- Colors (RGBA) -----
        # PVC-style red and white
        self.COLOR_GREEN_START = (0.0, 1.0, 0.0, 1.0)
        self.COLOR_PVC_RED     = (1.0, 0.0, 0.0, 1.0)
        self.COLOR_PVC_WHITE   = (1.0, 1.0, 1.0, 1.0)
        self.COLOR_LANDMARK    = (1.0, 1.0, 1.0, 1.0)

    def make_marker(self, mid, mtype, xyz, scale, color, ns):
        m = Marker()
        m.header = Header()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map'

        m.ns = ns
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD

        m.pose.position.x = xyz[0]
        m.pose.position.y = xyz[1]
        m.pose.position.z = xyz[2]
        m.pose.orientation.w = 1.0

        m.scale.x = scale[0]
        m.scale.y = scale[1]
        m.scale.z = scale[2]

        r, g, b, a = color
        m.color = ColorRGBA()
        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = a

        m.lifetime.sec = 0
        return m

    def timer_callback(self):
        markers = MarkerArray()
        mid = 0

        # ----- Start cube (green) -----
        markers.markers.append(
            self.make_marker(
                mid,
                Marker.CUBE,
                (self.start[0], self.start[1], 0.2),
                (0.3, 0.3, 0.3),
                self.COLOR_GREEN_START,
                'start'
            )
        )
        mid += 1

        # ----- Landmarks (white spheres) -----
        for (x, y) in self.landmarks:
            markers.markers.append(
                self.make_marker(
                    mid,
                    Marker.SPHERE,
                    (x, y, 0.4),
                    (0.4, 0.4, 0.4),
                    self.COLOR_LANDMARK,
                    'landmark'
                )
            )
            mid += 1

        # ----- Gate posts (red PVC cylinders) -----
        for (x, y) in self.gate_posts:
            markers.markers.append(
                self.make_marker(
                    mid,
                    Marker.CYLINDER,
                    (x, y, 0.75),          # center at mid-height
                    (0.15, 0.15, 1.5),     # radius_x, radius_y, height
                    self.COLOR_PVC_RED,
                    'gate_posts'
                )
            )
            mid += 1

        # ----- Gate crossbar (white PVC cylinder) -----
        markers.markers.append(
            self.make_marker(
                mid,
                Marker.CYLINDER,
                (-9.0, 0.0, 1.5),         # above the posts
                (0.1, 0.1, 2.2),          # thin long bar
                self.COLOR_PVC_WHITE,
                'gate_crossbar'
            )
        )
        mid += 1

        # ----- Channel bars (red PVC beams) -----
        for (x, y) in self.channel_bars:
            markers.markers.append(
                self.make_marker(
                    mid,
                    Marker.CUBE,
                    (x, y, 0.5),
                    (2.0, 0.3, 0.2),
                    self.COLOR_PVC_RED,
                    'channel_bars'
                )
            )
            mid += 1

        self.pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = EnvironmentMarkers()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
