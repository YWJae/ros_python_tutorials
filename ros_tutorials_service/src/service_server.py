#!/usr/bin/env python
import rospy
from ros_python_service.srv import SrvTutorial, SrvTutorialResponse # Created Path:~/catkin_ws/devel/include/ros_python_service/


# The below process is performed when there is a service request
# The service request is declared as 'req', and the service response is declared as 'res'
def calculation(req):
    # The service name is 'ros_python_srv' and it will call 'calculation' function upon the service request.
    result = req.a + req.b

    # Displays 'a' and 'b' values used in the service request and
    # the 'result' value corresponding to the service response
    rospy.loginfo("request: x=%ld, y=%ld" % (req.a, req.b))
    rospy.loginfo("sending back response: %ld" % result)

    # Thils 'SrvTutorialResponse' is automatically made with 'SrvTutorial'
    return SrvTutorialResponse(result)

def start_server():
    rospy.init_node('service_server')    # Initializes Node Name
    
    # Declare service server 'ros_python_server'
    # using the 'SrvTutorial' service file in the 'ros_python_service' package.
    # The service name is 'ros_python_srv' and it will call 'calculation' function
    # upon the service request.
    ros_python_server = rospy.Service('ros_tutorials_srv', SrvTutorial, calculation)
    
    rospy.loginfo("ready srv server")
    
    rospy.spin()    # Wait for the service request

if __name__ == "__main__":
    start_server()
