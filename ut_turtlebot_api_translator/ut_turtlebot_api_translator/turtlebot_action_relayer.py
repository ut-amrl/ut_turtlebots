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

# from action_tutorials_interfaces.action import Fibonacci

class ActionRelayer():
    def __init__(self, ros_node, msgType, base_action_name):
        self.msgType = msgType
        self._action_client = ActionClient(ros_node, msgType, "/" + base_action_name)
        self._action_server = ActionServer(ros_node, msgType, "/ut/" + base_action_name, self.execute_callback, cancel_callback=self.cancel_callback, callback_group=ReentrantCallbackGroup())  # This allows concurrent handling of requests)

    async def execute_callback(self, goal_handle):

        # Send goal to CustomActionType2 server
        while not  self._action_client.wait_for_server(timeout_sec=0.5):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return msgType.Result()

        self._action_client_goal_handle = await self._action_client.send_goal_async(goal_handle.request, feedback_callback=lambda feedback: self.feedback_callback(feedback, goal_handle))

        result_response_future = self._action_client_goal_handle.get_result_async()

        # Handle the response asynchronously
        result_response = await self._action_client_goal_handle.get_result_async()

        if GoalStatus.STATUS_SUCCEEDED:
            goal_handle.succeed()
            return result_response.result
        else:
            goal_handle.abort()
            return result_response.result


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

class ActionRelayerNode(Node):

    def __init__(self, base_names_to_msg_type_dicts):
        super().__init__('action_relayer_node')
        self._action_relayer_dict = {}
        for base_name, msg_type in base_names_to_msg_type_dicts.items():
            self._action_relayer_dict[base_name] = ActionRelayer(self, msg_type, base_name)


def main(args=None):

    base_names_to_msg_type_dicts = {
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

    # # For testing
    # base_names_to_msg_type_dicts = {
    #         "fibonacci": Fibonacci,
    #         "fibonacci2": Fibonacci
    # }

    rclpy.init(args=args)
    action_adapter_node = ActionRelayerNode(base_names_to_msg_type_dicts=base_names_to_msg_type_dicts)
    rclpy.spin(action_adapter_node)
    action_adapter_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
