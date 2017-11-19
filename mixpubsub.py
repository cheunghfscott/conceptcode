#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
	rospy.loginfo("heard %s", data.data)
#	talker()
	pub= rospy.Publisher('message2',String,queue_size=10)
        rate=rospy.Rate(10)
        mess= "heard2 %s" % data.data
        rospy.loginfo(mess)
        pub.publish(mess)
        rate.sleep()


def listner():
	rospy.init_node('PubSub',anonymous=True)
	rospy.Subscriber('message',String,callback)

def talker():
	pub= rospy.Publisher('message2',String,queue_size=10)
	rate=rospy.Rate(10)
	mess= "heard2 %s" % data.data
	rospy.loginfo(mess)
	pub.publish(mess)
	rate.sleep()

if __name__=='__main__':
	listner()
#	try:
#	talker()
	rospy.spin()
#	except rospy.ROSInterruptException:
#		pass


