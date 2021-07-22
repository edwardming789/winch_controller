import rospy
from winch_controller.msg import Depth, Command
from winch_controller.srv import SetDepth, SetVelocity

class Controller:
    def __init__(self):
        self.depth_sub = rospy.Subscriber("/simulator/depth", Depth, self.depth_sub_callback)
        self.set_depth_service = rospy.Service("simulator/set_depth", SetDepth, self.set_depth_callback)
        #self.set_vel_client = rospy.ServiceProxy("simulator/set_velocity", SetVelocity)
        self.command_pub = rospy.Publisher('simulator/command', Command, queue_size=10)

        #self.winch_control_loop_timer = rospy.Timer(rospy.Duration(0.1),self.test)

        self._current_depth = 0
        self._target_depth = 0
        #self._set_vel_request = SetVelocity()
        self._command = Command()
        #self._winch_command = Command()

    def depth_sub_callback(self,depth):
        self._current_depth = depth.depth
        #print(self._current_depth)

    def set_depth_callback(self,req):
        try:
            self._target_depth = req.depth
            rospy.loginfo("target depth set to: %f", self._target_depth)
            return True
        except:
            return False

    # def set_velocity(self):
    #     self.set_vel_client.wait_for_service()
    #     try:
    #         resp = self.set_vel_client(self._set_vel_request)
    #     except rospy.ServiceException as e:
    #         print("Service call failed: %s"%e)

        #print(self._winch_command.vel)
        #self.command_pub.publish(self._winch_command)
    
    def main_loop(self):

        if self._target_depth - 1.0 < self._current_depth < self._target_depth + 1.0:
            self._command.vel = 0.0
            rospy.loginfo("winch stop")
        elif self._target_depth >= self._current_depth + 1.0:
            self._command.vel = -0.3566
            rospy.loginfo("winch down")
        else:
            self._command.vel = 0.3302
            rospy.loginfo("winch up")
        self.command_pub.publish(self._command)
       

if __name__ == '__main__':

    rospy.init_node('winch_controller')
    winch_controller = Controller()
    
    #rospy.spin()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        winch_controller.main_loop()
        rate.sleep()
