#!/usr/bin/env python
import tf
import rospy
import sys
import math
import numpy as np
import time
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Transform, Vector3, Twist
from std_msgs.msg import Int32, Int32MultiArray
import exp_quat_func as eqf
import ar_tag_subs as eqs

#set the global variables to be the correct starting values
listener = None
prev_vel = None
stop1 = True
stop2 = True
passed = False
collected = False
aligned = False
Rval = float('inf')
Gval = float('inf')
Bval = float('inf')
Oval = float('inf')

stopTwist = Twist()
stopTwist.angular.x = 0
stopTwist.angular.y = 0
stopTwist.angular.z = 0
stopTwist.linear.x = 0
stopTwist.linear.y = 0
stopTwist.linear.z = 0
counter = 0

def callback_red(message):
	#print 'red'
	global Rval
	Rval = message.data

def callback_blue(message):
	#print 'blue'
	global Bval
	Bval = message.data

def callback_green(message):
	#print 'green'
	global Gval
	Gval = message.data

def callback_orange(message):
	#print 'orange'
	global Oval
	Oval = message.data

#in order to change which zumy moves first, simply switch the order of zumy with zumy2
def run(zumy, zumy2, ar_tags):
	global prev_vel, stop1, stop2, passed, collected, aligned
	ON = 1
	OFF = 0
	listener = tf.TransformListener()
	zumy_vel = rospy.Publisher('%s/cmd_vel' % zumy, Twist, queue_size=2)
	zumy_vel2 = rospy.Publisher('%s/cmd_vel' % zumy2, Twist, queue_size=2)
	em1 = rospy.Publisher('/' + zumy + '/digital_out', Int32, queue_size=2)
	em2 = rospy.Publisher('/' + zumy2 + '/digital_out', Int32, queue_size=2)
	red    = rospy.Subscriber("/zumy/red", Int32MultiArray, callback_red, queue_size=1)
	blue   = rospy.Subscriber("/zumy/blue", Int32MultiArray, callback_blue, queue_size=1)
	green  = rospy.Subscriber("/zumy/green", Int32MultiArray, callback_green, queue_size=1)
	orange = rospy.Subscriber("/zumy/orange", Int32MultiArray, callback_orange, queue_size=1)

	rate = rospy.Rate(10)
	print ar_tags

	#update this to be data = 1 for the actual project
	for i in range(10):
		em1.publish(data=1)
		em2.publish(data=0)
		rate.sleep()

	while not rospy.is_shutdown():
		print('begin while loop')
		if aligned == False:
			aligned = align_zumys(zumy,zumy2,zumy_vel,zumy_vel2,ar_tags)
			# print aligned

		if aligned == False:
			goForward(zumy_vel, zumy_vel2)

		# pass
		if aligned == True and passed == False:
			passBall(zumy_vel, zumy_vel2, em1, em2, rate)
			passed = True

		if aligned == True and passed == True:
			delay5secs()

		# receive the ball if there is a miss
		if aligned == True and passed == True and collected == False:
			print('fourth if')
			global counter
			counter += 1
			print(counter)

			dist_threshold = 80
			curr_m = -1
			dist = float("inf")
			# while curr_m != 1:
			# 	(curr_m, dist) = find_ball(zumy_vel2,Rval,Gval,Bval,Oval)
			# print 'facing ball'

			while dist > dist_threshold or curr_m != 1:
				(curr_m, dist) = find_ball(zumy_vel2,Rval,Gval,Bval,Oval)
				# delay()
				rospy.sleep(1.75)
			for _ in range(10):
				em2.publish(data=1)
			collected = True
			delay5secs()
			for _ in range(10):
				em2.publish(data=0)

		if collected:
			zumy_vel2.publish(stopTwist)

		rate.sleep()

def align_zumys(zumy,zumy2,zumy_vel,zumy_vel2,ar_tags):
	print 'aligning'
	stop1,stop2,not_aligned = False,False,True
	listener = tf.TransformListener()
	rate = rospy.Rate(5)

	for i in range(10):
		try:
			(trans, rot) = listener.lookupTransform(ar_tags[zumy], ar_tags[zumy2], rospy.Time(0))
			(ob1_trans,ob1_rot) = listener.lookupTransform(ar_tags[zumy], ar_tags['obstacle'],rospy.Time(0))
			(ob2_trans,ob2_rot) = listener.lookupTransform(ar_tags['obstacle'],ar_tags[zumy2], rospy.Time(0))

			print 'TRANS: ' + str(trans[0]) + ',' + str(trans[1])
			print 'OBSTACLE_ZUMY: '  + str(ob1_trans[0]+ob2_trans[0]) + ',' + str(ob1_trans[1]-ob2_trans[1])

			if (abs(trans[1] - (ob1_trans[1]-ob2_trans[1])) < 0.3):
				print 'not able to pass'
				return False
		except:
			short_delay()

	while not_aligned:
		if stop1 == False:
			print('first if')
			try:
				(trans, rot) = listener.lookupTransform(ar_tags[zumy2], ar_tags[zumy], rospy.Time(0))
			except:
				continue

			(omega, theta) = eqf.quaternion_to_exp(np.array(rot))
			v = eqf.find_v(omega,theta,trans)

			twist = Twist()

			twist.linear.x = v[0][0] 
			twist.angular.z = omega[2] 

			xdelta = 0.06
			ydelta = 0.5
			delta = .2
			minspeed = 0.15
			maxspeed = 0.3
			m = 0.36
			b = 0.07
			angularfactor = 0.15

			twist.angular.x = 0
			twist.angular.y = 0
			twist.linear.y = 0
			twist.linear.z = 0
			
			
			if abs(trans[0]) + abs(trans[1]) < delta:
				print("State: STOP")
				twist.linear.x = 0
				twist.angular.z = 0
			elif abs(trans[0]) < xdelta and trans[1] < 0:
				print("State: Forward")
				twist.angular.z = 0
				twist.linear.x = 0
				stop1 = True
			else:
				print("State: Turn")
				print 'TRANS_X: ' + str(trans[0])
				print 'TRANS_Y: ' + str(trans[1])

				twist.linear.x = 0
				factor = 0.2/abs(trans[0]*trans[1])
				twist.angular.z = factor * abs(trans[0] * trans[1]);

			zumy_vel2.publish(twist)
		
		#repeat for other zumy so that they both turn to each other
		if stop2 == False and stop1 == True:
			print('second if')
			try:
				(trans, rot) = listener.lookupTransform(ar_tags[zumy], ar_tags[zumy2], rospy.Time(0))
			except:
				continue

			(omega, theta) = eqf.quaternion_to_exp(np.array(rot))
			v = eqf.find_v(omega,theta,trans)

			twist2 = Twist()

			twist2.linear.x = v[0][0] 
			twist2.angular.z = omega[2]

			twist2.angular.x = 0
			twist2.angular.y = 0
			twist2.linear.y = 0
			twist2.linear.z = 0

			xdelta = 0.03
			ydelta = 0.5
			delta = .2
			minspeed = 0.15
			maxspeed = 0.3
			m = 0.36
			b = 0.07
			angularfactor = 0.1
			
			
			if abs(trans[0]) + abs(trans[1]) < delta:
				print("State: STOP")
				twist2.linear.x = 0
				twist2.angular.z = 0
			elif abs(trans[0]) < xdelta and trans[1] < 0:
				print("State: Forward")
				twist2.angular.z = 0
				twist2.linear.x = 0
				stop2 = True
			else:
				print("State: Turn")
				print 'TRANS_X: ' + str(trans[0])
				print 'TRANS_Y: ' + str(trans[1])
				twist2.linear.x = 0
				factor = 0.15/abs(trans[0]*trans[1])
				twist2.angular.z = factor * abs(trans[0] * trans[1]);

			zumy_vel.publish(twist2)

		if stop1 is True and stop2 is True: 
			print 'zumys aligned'
			not_aligned = False
			break

		rate.sleep()

	return True

def goForward(zumy_vel, zumy_vel2):
	twist = Twist()
	twist.angular.x = 0
	twist.angular.y = 0
	twist.angular.z = 0
	twist.linear.x = -0.10
	twist.linear.y = 0
	twist.linear.z = 0
	zumy_vel.publish(twist)
	twist.linear.x = -0.10
	zumy_vel2.publish(twist)

	delay()

	twist.linear.x = 0
	zumy_vel.publish(twist)
	zumy_vel2.publish(twist)

	delay5secs()
	print "both zumy's advance forward"

def passBall(zumy_vel, zumy_vel2, em1, em2, rate):
	for i in range(10):
	    em2.publish(data=1)
	    rate.sleep()

	for i in range(10):
	    em1.publish(data=0)
	    rate.sleep()

	print "\n passing the ball \n"
	passing_speed = 0.2

	#pub = rospy.Publisher('/zumy7a/digital_out', Int32, queue_size=1)
	
	twist = Twist()
	twist.angular.x = 0
	twist.angular.y = 0
	twist.angular.z = 0
	twist.linear.x = -passing_speed
	twist.linear.y = 0
	twist.linear.z = 0
	zumy_vel.publish(twist)
	rate.sleep()
	zumy_vel.publish(twist)

	short_delay()

	twist.linear.x = 0
	zumy_vel.publish(twist)
	zumy_vel2.publish(twist)

	print "passed the ball"

def find_ball(zumy_vel, R, G, B, O):
	print("beginning find ball")
	# - - - - - - - - - - - - - - #
	# Find Path to Ball.
	#
	#		B
	#	G 		O
	#
	# Ball Color: R
	# - - - - - - - - - - - - - - #

	global stopTwist
	twist = Twist()
	twist.angular.x = 0
	twist.angular.y = 0
	twist.angular.z = 0
	twist.linear.x = 0
	twist.linear.y = 0
	twist.linear.z = 0

	RG = sum([(R[0]-G[0])**2,(R[1]-G[1])**2])**0.5
	RB = sum([(R[0]-B[0])**2,(R[1]-B[1])**2])**0.5
	RO = sum([(R[0]-O[0])**2,(R[1]-O[1])**2])**0.5

	dist = {0:RG,1:RB,2:RO}
	m = min(dist,key = dist.get)
	if m == 0:
		# Min: Green
		# twist.angular.z = .2 #+ ((RO-RG)/2000)
		# twist.angular.x = 0
		for i in range(10):
			zumy_vel.publish(Twist(Vector3(0,0,0),Vector3(0,0,0.2-(i/100.0))))

	elif m == 1:
		# Min: Blue
		twist.angular.z = 0
		if RB > 150:
			twist.linear.x = -.135 #- RB/1000
		else:
			twist.linear.x = -.12
		for _ in range(10):
			zumy_vel.publish(twist)

	elif m == 2:
		# Min: Orange
		# twist.angular.z = -.2 #- ((RG-RO)/2000)
		# twist.angular.x = 0
		for i in range(10):
			zumy_vel.publish(Twist(Vector3(0,0,0),Vector3(0,0,-0.2+(i/100.0))))
	else:
		raise Exception('Impossible To Reach Here.')

	print(m, dist[m])
	print 'RG: {0}, RB: {1}, RO: {2}'.format(RG,RB,RO)

	# delay()
	short_delay()
	for _ in range(10):
		zumy_vel.publish(stopTwist)
	return (m, dist[m])

def delay():
	curr_c = time.clock()
	while time.clock()-curr_c < 1:
		# print '.'
		continue

def short_delay():
	curr_c = time.clock()
	while time.clock()-curr_c < 0.1:
		# print '.'
		continue

def delay5secs():
	curr_c = time.clock()
	while time.clock()-curr_c < 5:
		# print '.'
		continue
 
if __name__=='__main__':
	rospy.init_node('follow_ar_tag_twist')
	if len(sys.argv) < 6:
		print('Use: robocup.py [ zumy name with ball] [zumy receiving initially]\
			 [ AR tag number for Zumy with ball] [ AR tag number for receiving Zumy] \
			 [AR tag of obstacle]')
		sys.exit()
	ar_tags = {}
	zumy_name_with_ball = sys.argv[1]
	zumy_name = sys.argv[2]
	ar_tags[zumy_name_with_ball] = 'ar_marker_' + sys.argv[3]
	ar_tags[zumy_name] = 'ar_marker_' + sys.argv[4]
	ar_tags['obstacle'] = 'ar_marker_' + sys.argv[5]

	run(zumy=zumy_name_with_ball, zumy2=zumy_name, ar_tags=ar_tags)
	# passing(zumy=zumy_name_with_ball, zumy2=zumy_name, ar_tags=ar_tags)
	
	rospy.spin()