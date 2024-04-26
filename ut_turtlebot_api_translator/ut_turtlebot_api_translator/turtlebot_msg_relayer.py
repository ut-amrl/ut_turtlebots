import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist

class MsgRelayer():
    def __init__(self, node, base_name, ut_is_output, msg_data_type):

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=100)
        ut_topic = "/ut/" + base_name
        turtlebot_topic = "/" + base_name
        if (ut_is_output):
            pub_topic =  ut_topic
            sub_topic =  turtlebot_topic
        else:
            pub_topic = turtlebot_topic
            sub_topic = ut_topic
        self.topic_publisher = node.create_publisher(msg_data_type, pub_topic, qos)
        self.topic_subscription = node.create_subscription(msg_data_type, sub_topic, self.msgCallback,  qos)

    def msgCallback(self, msg):
        self.topic_publisher.publish(msg)


class RelayerNode(Node):

    def __init__(self, base_names_to_msg_types):
        super().__init__('ut_msg_relayer_node')
        self._msg_relayer_list = []

        # ut_is_output=True means that the publisher should be prefixed with "/ut", otherwise the subscriber should be prefixed with "/ut"
        for base_name, ut_is_output, msg_data_type in base_names_to_msg_types:
            self._msg_relayer_list.append(MsgRelayer(self, base_name, ut_is_output, msg_data_type))


def main(args=None):

    base_names_to_msg_types = [
        ("cmd_vel", False, Twist)
    ]
    rclpy.init(args=args)
    adapter_node = RelayerNode(base_names_to_msg_types=base_names_to_msg_types)
    
    returnVal = False
    try:
        rclpy.spin(adapter_node)
        adapter_node.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        # Handle exceptions appropriately.
        adapter_node.get_logger().error(f"Spin failed: {str(e)}")
        print("About to return true to indicate that we should restart")
        returnVal = True
    finally:
        adapter_node.destroy_node()
        rclpy.shutdown()
        print("REturning ", returnVal)
        return returnVal

if __name__ == '__main__':
    mainResult = main()
    print("Main completed with: ", mainResult)
    while (mainResult):
        print("Restarting node")
        mainResult = main()
