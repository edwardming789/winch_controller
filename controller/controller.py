import rospy
from winch_controller.msg import Depth, Command
from winch_controller.srv import SetDepth, SetVelocity

class Controller:
    def __init__(self):
        self.depth_sub = rospy.Subscriber("/simulator/depth", Depth, self.depth_sub_callback)
        self.set_depth_service = rospy.Service("simulator/set_depth", SetDepth, self.set_depth_callback)
        self.command_pub = rospy.Publisher('simulator/command', Command, queue_size=10)


        self._current_depth = 0
        self._target_depth = 0
        self._command = Command()
        self._initialise = True
        self._stop_winch =False

    def depth_sub_callback(self,depth):
        self._current_depth = depth.depth

    def set_depth_callback(self,req):
        try:
            self._target_depth = req.depth
            rospy.loginfo("target depth set to: %f", self._target_depth)
            return True
        except:
            return False

    def initialise(self):
        rospy.loginfo("winch waiting for target depth")
        while not rospy.is_shutdown():
            if self._target_depth != self._current_depth:
                break
    
    def winch_down(self):
        self._start_time = rospy.get_rostime()

        while self._target_depth >= self._current_depth + 1.0:
            rospy.loginfo("winch down, target depth: %.0f, current depth: %.0f", self._target_depth, self._current_depth)
            if self._start_time + rospy.Duration(5) > rospy.get_rostime():
                self._command.vel = -0.3566
                self.command_pub.publish(self._command)
            else:
                rospy.loginfo("winch stop after 20 seconds")
                self._stop_winch = True
                break

    def winch_up(self):
        self._start_time = rospy.get_rostime()
        self._time_increment = rospy.get_rostime()
        
        self._start_depth = self._current_depth

        while self._target_depth >= self._current_depth + 1.0:
            rospy.loginfo("winch down, target depth: %.0f, current depth: %.0f", self._target_depth, self._current_depth)

            #check if sensor's depth decreases 1 meter every 5 seconds
            if self._time_increment + rospy.Duration(5) > rospy.get_rostime():
                if self._current_depth < self._start_depth - 1.0:
                    self._start_depth = self._current_depth
                else:
                    rospy.loginfo("winch stuck, stopping operation")
                    self._stop_winch = True
                    break
                self._time_increment += rospy.Duration(5)

            #stop winch after 20 seconds
            if self._start_time + rospy.Duration(20) > rospy.get_rostime():
                self._command.vel = 0.3302
                self.command_pub.publish(self._command)
            else:
                rospy.loginfo("winch stop after 20 seconds")
                self._stop_winch = True
                break
            

    def set_velocity(self):
        if self._target_depth - 1.0 < self._current_depth < self._target_depth + 1.0:
            self._command.vel = 0.0
            rospy.loginfo("winch stop")
        elif self._target_depth >= self._current_depth + 1.0:
            self.winch_down()
        else:
            self.winch_up()
        #self.command_pub.publish(self._command)

    def main_loop(self):
        if self._initialise:
            self.initialise()
            self._initialise = False

        if not self._stop_winch:
            self.set_velocity()
        else:
            rospy.signal_shutdown("Stopping winch operation")

       

if __name__ == '__main__':

    rospy.init_node('winch_controller')
    winch_controller = Controller()
    
    #rospy.spin()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        winch_controller.main_loop()
        rate.sleep()
