#! /usr/bin/env python

import rospy

import actionlib

from ros_tutorials_action.msg import FibonacciAction, FibonacciFeedback, FibonacciResult

class Fibonacci(object):
    # create messages that are used to publish feedback/result
    _feedback = FibonacciFeedback()
    _result = FibonacciResult()


    # Initialize action server (action name, action callback function)
    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, FibonacciAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

      
    # A function that receives an action goal message and performs a specified
    # action (in this example, a Fibonacci calculation).
    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)    # Loop Rate: 1Hz
        success = True       # Used as a variable to store the success or failure of an action
        
        # Setting Fibonacci sequence initialization,
        # add first (0) and second message (1) of feedback.
        self._feedback.sequence = []
        self._feedback.sequence.append(0)
        self._feedback.sequence.append(1)
        
        # publish info to the console for the user
        rospy.loginfo('%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i' % (self._action_name, goal.order, self._feedback.sequence[0], self._feedback.sequence[1]))
        
        # start executing the action
        for i in range(1, goal.order):
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                # Action cancellation and consider action as failure and save to variable
                self._as.set_preempted()
                success = False
                break

            # Store the sum of current Fibonacci number and the previous number in the feedback
            # while there is no action cancellation or the action target value is reached.
            self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
            # publish the feedback
            self._as.publish_feedback(self._feedback)
            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep()
          

        # If the action target value is reached,
        # transmit current Fibonacci sequence as the result value.
        if success:
            self._result.sequence = self._feedback.sequence
            rospy.loginfo('%s: Succeeded' % self._action_name)
            # set the action state to succeeded
            self._as.set_succeeded(self._result)

        
if __name__ == '__main__':
    rospy.init_node('action_server') 		    # Initializes Node Name
    server = Fibonacci("ros_tutorials_action")   # Fibonacci Declaration (Action Name: ros_tutorial_action)
    rospy.spin() 				    # Wait to receive action goal
