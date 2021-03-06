import rospy

from ros_tcp_endpoint import *
from winch_controller.msg import Depth
from winch_controller.srv import WinchTrigger

def main():
    rospy.set_param("/ROS_IP", "127.0.0.1")
    rospy.set_param("/use_sim_time", "true")
    ros_node_name = rospy.get_param("/TCP_NODE_NAME", 'TCPServer')
    buffer_size = rospy.get_param("/TCP_BUFFER_SIZE", 1024)
    connections = rospy.get_param("/TCP_CONNECTIONS", 10)
    tcp_server = TcpServer(ros_node_name, buffer_size, connections)
    
    rospy.init_node(ros_node_name, anonymous=True)

    tcp_server.start({
        'simulator/depth': RosPublisher('simulator/depth', Depth, queue_size=1),
        'winch/winch_up': UnityService('winch/winch_up', WinchTrigger, tcp_server),
        'winch/winch_down': UnityService('winch/winch_down', WinchTrigger, tcp_server),
        'winch/winch_stop': UnityService('winch/winch_stop', WinchTrigger, tcp_server),
        #'simulator/set_velocity': UnityService('simulator/set_velocity', SetVelocity, tcp_server)
        #'simulator/command': RosSubscriber('simulator/command', Command, tcp_server)
    })

    rospy.spin()

if __name__ == "__main__":
    main()