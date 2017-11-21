#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt16MultiArray
from std_msgs.msg import UInt16

def callback(data):
	rospy.loginfo("heard %s", data.data)
#	talker()
	pub= rospy.Publisher('outputAng',UInt16MultiArray,queue_size=10)
        rate=rospy.Rate(10)
	#data.data[0] = data.data[0] +1
	#array=[]
	array=data.data
	array2=[]
	array2=[array[0]+1,array[1]+1]
	mess = UInt16MultiArray(data=array2)
        pub.publish(mess)
	rospy.loginfo("heard2 %s", mess.data)
        rate.sleep()


def listner():
	rospy.init_node('PubSub',anonymous=True)
	rospy.Subscriber('inputAng',UInt16MultiArray,callback)

def talker():
	pub= rospy.Publisher('outputAng',UInt16MultiArray,queue_size=10)
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


