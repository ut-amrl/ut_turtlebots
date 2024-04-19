import rclpy
from rclpy.action import CancelResponse
from action_msgs.msg import GoalStatus
from rclpy.action import ActionServer
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from irobot_create_msgs.action import Dock
from irobot_create_msgs.msg import DockStatus

from amrl_msgs.action import TurtlebotDock
from amrl_msgs.msg import TurtlebotDockStatus

class DockActionTranslator():
    def __init__(self, ros_node, base_action_name="dock"):
        self._action_client = ActionClient(ros_node, Dock, "/" + base_action_name)
        self._action_server = ActionServer(ros_node, TurtlebotDock, "/ut/turtlebot_" + base_action_name, self.execute_callback, cancel_callback=self.cancel_callback, callback_group=ReentrantCallbackGroup())  # This allows concurrent handling of requests)

    async def execute_callback(self, goal_handle):

        # Send goal to CustomActionType2 server
        while not self._action_client.wait_for_server(timeout_sec=0.5):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return TurtlebotDock.Result()
            
        turtlebotRequest = TurtlebotDock.Goal()

        self._action_client_goal_handle = await self._action_client.send_goal_async(turtlebotRequest, feedback_callback=lambda feedback: self.feedback_callback(feedback, goal_handle))

        # Handle the response asynchronously
        result_response = await self._action_client_goal_handle.get_result_async()

        if GoalStatus.STATUS_SUCCEEDED:
            goal_handle.succeed()
            turtlebotDockResult = TurtlebotDock.Result()
            turtlebotDockResult.is_docked = result_response.result.is_docked

            return turtlebotDockResult
        else:
            goal_handle.abort()
            return TurtlebotDock.Result()


    def feedback_callback(self, feedback_msg, goal_handle):
        turtlebotDockFeedback = TurtlebotDock.Feedback()
        turtlebotDockFeedback.sees_dock = feedback_msg.sees_dock
        goal_handle.publish_feedback(turtlebotDockFeedback)

    async def cancel_callback(self, goal_handle):
        # TODO Not clear if cancelling works correctly -- not much in terms of examples/debugging guidance
        if goal_handle.is_active:
            if hasattr(self, '_action_client_goal_handle') and self._action_client_goal_handle:
                cancel_response = await self._action_client_goal_handle.cancel_goal_async()
                # Check the response to see if the goal was actually canceled
                if cancel_response.return_code == 0: # ERROR_NONE: https://docs.ros2.org/galactic/api/action_msgs/srv/CancelGoal.html
                    return CancelResponse.ACCEPT
        return CancelResponse.REJECT


class TurtlebotDockTranslatorNode(Node):

    def __init__(self):
        super().__init__('turtlebot_dock_translator')
        self.dock_action_translator = DockActionTranslator(self)
        self.dock_status_publisher = self.create_publisher(TurtlebotDockStatus, '/ut/dock_status', 10)
        self.dock_status_subscription = self.create_subscription(DockStatus, '/dock_status', self.turtlebotFormatDockStatusCallback, 10)

    def turtlebotFormatDockStatusCallback(self, msg):
        turtlebotDockStatus = TurtlebotDockStatus()
        turtlebotDockStatus.is_docked = msg.is_docked
        turtlebotDockStatus.dock_visible = msg.dock_visible
        turtlebotDockStatus.header = msg.header
        self.dock_status_publisher.publish(turtlebotDockStatus)



def main(args=None):

    rclpy.init(args=args)
    action_adapter_node = TurtlebotDockTranslatorNode()
    rclpy.spin(action_adapter_node)
    action_adapter_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
