import rospy
from enum import Enum
from lab.srv import UpdateMode

class Color(Enum):
	red = 1
	green = 2
	blue = 3 

class joy():
	def __init__(self):
		self.color = Color.red
		#rospy.wait_for_service("/change_mode")
		self._changemode = rospy.ServiceProxy("/change_mode", UpdateMode)
	def get_color(self):
		return self.color.name
	def set_color(self, value):
		self.color = Color[value]

if __name__ == '__main__':
	rospy.init_node('tmp')
	tmp = joy()
	rospy.loginfo(tmp.get_color())
	if (tmp.get_color() == 'red'):
		rospy.loginfo("change it!")
		tmp._changemode(["blue"])
		rospy.loginfo(tmp.color)
