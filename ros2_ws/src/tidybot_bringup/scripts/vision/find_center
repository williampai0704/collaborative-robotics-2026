# ROS publisher code for finding the center coord of the bbox
# Bbox provided from the SAM 3 block
# Subscribe to the SAM3 block

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


# uncomment depending on SAM3 output type

# CASE 1:
# If SAM3 publishes [xmin, ymin, xmax, ymax]
# as std_msgs/msg/Int32MultiArray
# ------------------------------------------------------------
from std_msgs.msg import Int32MultiArray
INPUT_MSG_TYPE = Int32MultiArray
# change the input topic name as needed
INPUT_TOPIC = '/sam3/bbox'
MODE = 'xyxy_array'

# ------------------------------------------------------------
# CASE 2:
# If SAM3 publishes sensor_msgs/msg/RegionOfInterest
# (x_offset, y_offset, width, height)
# ------------------------------------------------------------
# from sensor_msgs.msg import RegionOfInterest
# INPUT_MSG_TYPE = RegionOfInterest
# INPUT_TOPIC = '/sam3/bbox'
# MODE = 'roi'

# ------------------------------------------------------------
# CASE 3:
# If SAM3 publishes vision_msgs/msg/Detection2DArray
# ------------------------------------------------------------
# from vision_msgs.msg import Detection2DArray
# INPUT_MSG_TYPE = Detection2DArray
# INPUT_TOPIC = '/sam3/bboxes'
# MODE = 'det2d'



class FindCenter(Node):
    def __init__(self):
        super().__init__('find_center')

        # current publishing topic name: /bbox_center 
        self.center_pub = self.create_publisher(Point, '/bbox_center', 10)
        self.sub = self.create_subscription(INPUT_MSG_TYPE, INPUT_TOPIC, self.cb, 10)

        self.get_logger().info(f"find_center started. MODE={MODE}, subscribing to {INPUT_TOPIC}")

    def cb(self, msg):
        if MODE == 'xyxy_array':
            if len(msg.data) < 4:
                self.get_logger().warn(f"Expected 4 values [xmin,ymin,xmax,ymax], got {len(msg.data)}")
                return
            xmin, ymin, xmax, ymax = msg.data[:4]
            cx = 0.5 * (xmin + xmax)
            cy = 0.5 * (ymin + ymax)
        # calculating for different input types just incase
        elif MODE == 'roi':
            cx = msg.x_offset + 0.5 * msg.width
            cy = msg.y_offset + 0.5 * msg.height

        elif MODE == 'det2d':
            if not msg.detections:
                self.get_logger().debug("No detections.")
                return
            bbox = msg.detections[0].bbox
            cx = bbox.center.position.x
            cy = bbox.center.position.y

        else:
            self.get_logger().error(f"Unknown MODE={MODE}")
            return

        out = Point()
        out.x = float(cx)
        out.y = float(cy)
        out.z = 0.0

        self.center_pub.publish(out)
        self.get_logger().info(f"Center: ({out.x:.2f}, {out.y:.2f})")


def main():
    rclpy.init()
    node = FindCenter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
