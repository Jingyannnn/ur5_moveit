#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray
class JointState2Gazebo:
	def __init__(self):
		self.joint_state = JointState()
		
		self.jp1 =rospy.Publisher('/joint_controller_ur1/command',Float64, queue_size=1)
		self.jp2 =rospy.Publisher('/joint_controller_ur2/command',Float64, queue_size=1)
		self.jp3 =rospy.Publisher('/joint_controller_ur3/command',Float64, queue_size=1)
		self.jp4 =rospy.Publisher('/joint_controller_ur4/command',Float64, queue_size=1)
		self.jp5 =rospy.Publisher('/joint_controller_ur5/command',Float64, queue_size=1)				
		self.jp6 =rospy.Publisher('/joint_controller_ur6/command',Float64, queue_size=1)
		self.bag_sub = rospy.Subscriber("/joint_states", JointState, self.states_callback, queue_size=1)

	
	def states_callback(self, msg):

		self.joint_state.header.stamp = rospy.Time.now()
		self.joint_state.position = msg.position
		self.joint_state.name = msg.name

		for i, j in zip(msg.position,  msg.name):
			if j=="shoulder_pan_joint":
				self.jp1.publish(i)
			elif j=="shoulder_lift_joint":
				self.jp2.publish(i)
			elif j=="elbow_joint":
				self.jp3.publish(i)		
			elif j=="wrist_1_joint":
				self.jp4.publish(i)		
			elif j=="wrist_2_joint":
				self.jp5.publish(i)
			elif j=="wrist_3_joint":
				self.jp6.publish(i)
			else:
				print("Joint Invalid")
				

if __name__ == '__main__':
    try:
        rospy.init_node('joint_state', anonymous=True)
        js_gazebo = JointState2Gazebo()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
