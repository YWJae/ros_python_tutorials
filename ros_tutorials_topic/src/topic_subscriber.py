#!/usr/bin/env python
import rospy					# ROS Default File
from ros_tutorials_topic.msg import MsgTutorial

def msgCallback(msg):
    rospy.loginfo("recieve msg = %d", msg.stamp.secs);   # Prints the 'stamp.sec' message
    rospy.loginfo("recieve msg = %d", msg.stamp.nsecs);  # Prints the 'stamp.nsec' message
    rospy.loginfo("recieve msg = %d", msg.data);        # Prints the 'data' message


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('topic_subscriber', anonymous=True)	# Initializes Node Name

    # Declares subscriber. Create subscriber 'ros_tutorial_sub' using the 'MsgTutorial'
    # message file from the 'ros_tutorials_topic' package. The topic name is 'ros_tutorial_msg'
    ros_tutorial_sub = rospy.Subscriber('ros_tutorial_msg', MsgTutorial, msgCallback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
