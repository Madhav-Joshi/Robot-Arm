from goemetry_msgs.msg import Pose
from std_msgs.msg import Bool
import roslib
roslib.load_manifest('master') 
import rospy
import actionlib

# subscriber for joint states,
# subsribe for final goal from CV --> forward_kin
# publish final goal and current pose to MATLAB




bool turining_state
bool scanning_state
bool homing_state
bool regionOne
bool regionTwo
bool regionThree


class Manager():

	def __init__(self):

		rospy.init()
		rospy.Subscriber('current_state',Pose, self.current_state)
		rospy.Subscriber('final_state', Pose, self.final_state)
		rospy.Subscriber('/car_detected', Bool, car_detect)
		rospy.Publisher()


		self.turining_state = 0
		self.scanning state = 0
		self.homing_state = 0




	def car_detect(self,msg):
		self.car_detected = True

	def manage_func(self):

		if turning_state:
			self.turning

		elif scanning state:
			self.scan 

		elif homing_state:
			#check region and redistribute.

	def CvClient():




if '__name__' =='__main__':

	manager = Manger()
