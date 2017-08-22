import rospy
import threading
import config
import string
from skeleton_tracker.msg import user_IDs

class userListenerThread(threading.Thread):
	def __init__(self):
		threading.Thread.__init__(self)
		rospy.Subscriber('people', user_IDs, self.callback)

	def run(self):
		rospy.spin()

	def callback(self, data):
		user_list = list(data.users)
		if user_list:
			config.user = ord(user_list[-1])
