#!/usr/bin/env python
import rospy					# ROS Default File
from ros_tutorials_topic.msg import MsgTutorial

def talker():
    rospy.init_node('topic_publisher', anonymous=True)	# Initializes Node Name

    # Declare publisher, create publisher 'ros_tutorial_pub' using the 'MsgTutorial'
    # message file from the 'ros_tutorials_topic' package. The topic name is
    # 'ros_tutorial_msg' and the size of the publisher queue is set to 100.
    ros_tutorial_pub = rospy.Publisher('ros_tutorial_msg', MsgTutorial, queue_size=100)

    # Set the loop period. '10' refers to 10 Hz and the main loop repeats at 0.1 second intervals
    loop_rate = rospy.Rate(10)

    msg = MsgTutorial()	  # Declares message 'msg' in 'MsgTutorial' message file format
    count = 0		  # Variable to be used in message

    while not rospy.is_shutdown():
        msg.stamp = rospy.Time.now()		# Save current time in the stamp of 'msg'
        msg.data = count			# Save the the 'count' value in the data of 'msg'

        rospy.loginfo("send msg = %d" % msg.stamp.secs)	    # Prints the 'stamp.sec' message
        rospy.loginfo("send msg = %d" % msg.stamp.nsecs)	    # Prints the 'stamp.nsec' message
        rospy.loginfo("send msg = %d" % msg.data)	    # Prints the 'data' message

        ros_tutorial_pub.publish(msg)		# Publishes 'msg' message
        loop_rate.sleep()			# Goes to sleep according to the loop rate defined above.
	
        count += 1				# Increase count variable by one


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
