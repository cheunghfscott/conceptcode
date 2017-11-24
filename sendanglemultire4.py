#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt16MultiArray
from std_msgs.msg import UInt16

def callback(data):
	rospy.loginfo("heard %s", data.data)
	array3=process(data)
#	talker()
	pub= rospy.Publisher('servo',UInt16MultiArray,queue_size=10)
        rate=rospy.Rate(20)
	mess = UInt16MultiArray(data=array3)
        pub.publish(mess)
	#rospy.loginfo("heard3 %s", mess.data)
        rate.sleep()

def process(data):
	array=data.data
	#rospy.loginfo("sending %s", array)
	array2=[]
	#array2=[array[0]+1,array[1]+1]
	array2=[array[0],array[1],array[2],array[3]]
	return array2

def listner():
	rospy.init_node('PubSub',anonymous=True)
	rospy.Subscriber('inputAng',UInt16MultiArray,callback)


if __name__=='__main__':
	listner()
#	try:
#	talker()
	rospy.spin()
#	except rospy.ROSInterruptException:
#		pass


