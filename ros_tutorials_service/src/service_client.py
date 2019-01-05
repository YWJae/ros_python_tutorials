#!/usr/bin/env python
import rospy                          		  # ROS Default File
from ros_python_service.srv import SrvTutorial  # SrvTutorial Service File Header (Automatically created after build)
import sys

def request_service(x, y):
    rospy.init_node('service_client')	        # Initializes Node Name
    rospy.wait_for_service('ros_tutorials_srv')
    try:
        # Declares service client 'service_client'
        # using the 'SrvTutorial' service file in the 'ros_python_service' package.
        # The service name is 'ros_python_srv'
        service_client = rospy.ServiceProxy('ros_tutorials_srv', SrvTutorial)
		
        # Request service
        response = service_client(x, y)
        
        rospy.loginfo("send srv, srv.Request.a and b: %ld, %ld" % (x, y))
        rospy.loginfo("receive srv, srv.Response.result: %ld" % response.result)

    except rospy.ServiceException, e:
        rospy.loginfo("Failed to call service ros_tutorial_srv");


if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:   # Input value error handling
        print("cmd : rosrun ros_tutorials_service service_client arg0 arg1")
        print("arg0: int number, arg1: int number")
        sys.exit(1)

    request_service(x, y)
