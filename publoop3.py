#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt16MultiArray
import yaml

with open('a.yaml') as fa:
		a=yaml.load(fa)
with open('b.yaml') as fb:
		b=yaml.load(fb)
with open('c.yaml') as fc:
		c=yaml.load(fc)
with open('d.yaml') as fd:
		d=yaml.load(fd)
order= 0

count = len(a)
def talker(order,count):
	pub= rospy.Publisher('inputAng',UInt16MultiArray,queue_size=10)
	rospy.init_node('PubLoop',anonymous=True)
	rate=rospy.Rate(20)
	
	while not rospy.is_shutdown():
		x=[a[order],b[order],c[order],d[order]]
		mess1= UInt16MultiArray(data=x)
		rospy.loginfo("heard0 %s", mess1.data)
		pub.publish(mess1)
		order=order +1
		
		rate.sleep()

if __name__=='__main__':

	try:
		talker(order,count)
		order=order +1
	except rospy.ROSInterruptException:
		pass


