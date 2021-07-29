import rospy
from winch_controller.msg import Depth, Command
from winch_controller.srv import SetDepth, WinchTrigger

class Controller:
    def __init__(self):
        self.depth_sub = rospy.Subscriber("/simulator/depth", Depth, self.depth_sub_callback)
        self.set_depth_service = rospy.Service("simulator/set_depth", SetDepth, self.set_depth_callback)
        #self.command_pub = rospy.Publisher('simulator/command', Command, queue_size=10)
        self.winch_up_service = rospy.ServiceProxy('winch/winch_up', WinchTrigger)
        self.winch_down_service = rospy.ServiceProxy('winch/winch_down', WinchTrigger)
        self.winch_stop_service = rospy.ServiceProxy('winch/winch_stop', WinchTrigger)


        self._current_depth = 0
        self._target_depth = 0
        self._command = Command()
        self._initialise = True
        self._stop_winch =False

        self._threshold = 0.5

        self._rate = rospy.Rate(10)

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
        #self._target_depth = self._current_depth
        while not rospy.is_shutdown():
            if self._target_depth != self._current_depth:
                return

    def winch_down(self):
        self._start_time = rospy.get_rostime()

        #rospy.wait_for_service('winch/winch_down')
        try:
            self.winch_down_service()
            
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s"%e)
            return

        while not rospy.is_shutdown():
            if self._target_depth <= self._current_depth:
                rospy.loginfo("winch target reached")
                return
            #rospy.loginfo("winch down, target depth: %.1f, current depth: %.1f", self._target_depth, self._current_depth)
            if self._start_time + rospy.Duration(20) < rospy.get_rostime():
                rospy.loginfo("winch stop after 20 seconds")
                self._stop_winch = True
                self.winch_stop()
                return
            #self._rate.sleep()
        

    def winch_up(self):
        self._start_time = rospy.get_rostime()
        self._time_increment = rospy.get_rostime()
        
        self._start_depth = self._current_depth
        rospy.wait_for_service('winch/winch_up')
        try:
            self.winch_up_service()
            
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s"%e)

        while not rospy.is_shutdown():
            if self._target_depth >= self._current_depth:
                break

            #check if sensor's depth decreases 1 meter every 5 seconds
            if self._time_increment + rospy.Duration(5) < rospy.get_rostime():
                rospy.loginfo("winch checking if is stuck")
                if self._current_depth < self._start_depth:
                    self._start_depth = self._current_depth
                else:
                    rospy.loginfo("winch stuck, stopping operation")
                    self._stop_winch = True
                    self.winch_stop()
                    break
                self._time_increment += rospy.Duration(5)

            #stop winch after 20 seconds
            if self._start_time + rospy.Duration(20) < rospy.get_rostime():
                rospy.loginfo("winch stop after 20 seconds")
                self._stop_winch = True
                self.winch_stop()
                break


    def winch_stop(self):
        self._stop_target_depth = self._target_depth
        rospy.wait_for_service('winch/winch_stop')
        try:
            self.winch_stop_service()

        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s"%e)

        while not rospy.is_shutdown():
            if self._stop_target_depth != self._target_depth:
                rospy.loginfo("winch receive new target depth")
                break
        

    def winch_run(self):
        if self._initialise:
            self.initialise()
            self._initialise = False
        #rospy.loginfo("looping")
        if self._target_depth >= self._current_depth + self._threshold:
            rospy.loginfo("winch down")
            self.winch_down()
        elif self._target_depth <= self._current_depth - self._threshold:
            rospy.loginfo("winch up")
            self.winch_up()
        else:
            rospy.loginfo("winch stop")
            self.winch_stop()

        if not self._stop_winch:
            self.winch_run()
        else:
            rospy.signal_shutdown("Stopping winch operation")


       

if __name__ == '__main__':

    rospy.init_node('winch_controller')
    winch_controller = Controller()

    # winch_controller.winch_run()
    
    # rospy.spin()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        winch_controller.winch_run()
        rate.sleep()
        #rospy.spin()
