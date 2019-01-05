#! /usr/bin/env python
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
from ros_python_action.msg import FibonacciAction, FibonacciGoal

def fibonacci_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('ros_tutorials_action', FibonacciAction)

    # Waits until the action server has started up and started
    # listening for goals.
    rospy.loginfo("Waiting for action server to start.")
    client.wait_for_server()

    # Creates a goal to send to the action server.
    rospy.loginfo("Action server started, sending goal.");
    goal = FibonacciGoal(order=20)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('action_client')
        result = fibonacci_client()
        print("Action finished:", ', '.join([str(n) for n in result.sequence]))
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
