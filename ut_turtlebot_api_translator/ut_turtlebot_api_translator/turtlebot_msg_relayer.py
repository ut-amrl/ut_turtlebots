import rclpy
from rclpy.action import CancelResponse
from action_msgs.msg import GoalStatus
from rclpy.action import ActionServer
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from irobot_create_msgs.action import AudioNoteSequence
from irobot_create_msgs.action import Dock
from irobot_create_msgs.action import DriveArc
from irobot_create_msgs.action import DriveDistance
from irobot_create_msgs.action import LedAnimation
from irobot_create_msgs.action import NavigateToPosition
from irobot_create_msgs.action import RotateAngle
from irobot_create_msgs.action import Undock
from irobot_create_msgs.action import WallFollow
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist

# from action_tutorials_interfaces.action import Fibonacci

class ActionRelayer():
    def __init__(self, ros_node, msgType, base_action_name):
        self.msgType = msgType
        self._action_client = ActionClient(ros_node, msgType, "/" + base_action_name)
        self._action_server = ActionServer(ros_node, msgType, "/ut/" + base_action_name, self.execute_callback, cancel_callback=self.cancel_callback, callback_group=ReentrantCallbackGroup())  # This allows concurrent handling of requests)

    async def execute_callback(self, goal_handle):
        try:
            # Send goal to CustomActionType2 server
            while not  self._action_client.wait_for_server(timeout_sec=0.5):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return msgType.Result()

            self._action_client_goal_handle = await self._action_client.send_goal_async(goal_handle.request, feedback_callback=lambda feedback: self.feedback_callback(feedback, goal_handle))

            result_response_future = self._action_client_goal_handle.get_result_async()

            # Handle the response asynchronously
            result_response = await self._action_client_goal_handle.get_result_async()

            if result_response.status == GoalStatus.STATUS_SUCCEEDED:
                goal_handle.succeed()
                return result_response.result
            else:
                goal_handle.abort()
                return result_response.result
        except Exception as e:
            # Handle exceptions appropriately.
            goal_handle.abort()
            self.get_logger().error(f"Failed during execute_callback: {str(e)}")
            return self.msgType.Result()  # Ensure a result type is returned even in case of failure.

    def feedback_callback(self, feedback_msg, goal_handle):
        goal_handle.publish_feedback(feedback_msg.feedback)

    async def cancel_callback(self, goal_handle):
        # TODO Not clear if cancelling works correctly -- not much in terms of examples/debugging guidance
        if goal_handle.is_active:
            if hasattr(self, '_action_client_goal_handle') and self._action_client_goal_handle:
                cancel_response = await self._action_client_goal_handle.cancel_goal_async()
                # Check the response to see if the goal was actually canceled
                if cancel_response.return_code == 0: # ERROR_NONE: https://docs.ros2.org/galactic/api/action_msgs/srv/CancelGoal.html
                    return CancelResponse.ACCEPT
        return CancelResponse.REJECT

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

    def __init__(self, base_names_to_action_type_dicts, base_names_to_msg_types):
        super().__init__('ut_relayer_node')
        self._action_relayer_dict = {}
        self._msg_relayer_list = []
        for base_name, action_type in base_names_to_action_type_dicts.items():
            self._action_relayer_dict[base_name] = ActionRelayer(self, action_type, base_name)

        for base_name, ut_is_output, msg_data_type in base_names_to_msg_types:
            self._msg_relayer_list.append(MsgRelayer(self, base_name, ut_is_output, msg_data_type))


def main(args=None):

    base_names_to_action_type_dicts = {
        "audio_note_sequence": AudioNoteSequence,
        "dock": Dock,
        "drive_arc": DriveArc,
        "drive_distance": DriveDistance,
        "led_animation": LedAnimation,
        "navigate_to_position": NavigateToPosition,
        "rotate_angle": RotateAngle,
        "undock": Undock,
        "wall_follow": WallFollow
    }

    base_names_to_msg_types = [
        ("cmd_vel", False, Twist)
    ]


    # # For testing
    # base_names_to_msg_type_dicts = {
    #         "fibonacci": Fibonacci,
    #         "fibonacci2": Fibonacci
    # }

    rclpy.init(args=args)
    adapter_node = RelayerNode(base_names_to_action_type_dicts=base_names_to_action_type_dicts,
                                            base_names_to_msg_types=base_names_to_msg_types)
    
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
