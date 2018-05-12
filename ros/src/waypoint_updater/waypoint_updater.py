#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from scipy.spatial import KDTree
from styx_msgs.msg import Lane, Waypoint

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
	# Initialize member variables before registering any callbacks
	# that might potentially test the variables.
	self.pose_msg = None
	self.base_waypoints_msg = None
	self.waypoints_cartesian = None
	self.waypoints_tree = None

        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
	


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self._loop()

    

    def _get_next_waypoint_index(self):
	car_x = self.pose_msg.pose.position.x
	car_y = self.pose_msg.pose.position.y

	rospy.loginfo('Attemting to use KD Tree')
	_ , closest_waypoint_index = self.waypoints_tree.query([car_x, car_y], 1)
	preceding_waypoint_index =  (closest_waypoint_index > 0) if  (closest_waypoint_index - 1) else  (len(self.waypoints_cartesian) - 1)

	closest_waypoint = self.waypoints_cartesian[closest_waypoint_index]
	preceding_waypoint = self.waypoints_cartesian[preceding_waypoint_index]

	car_vector = np.array([car_x, car_y])
	closest_wp_vector = np.array(closest_waypoint)
	preceding_wp_vector = np.array(preceding_waypoint)

	direction_test = np.dot(closest_wp_vector - preceding_wp_vector, car_vector - closest_wp_vector)

	if direction_test > 0:
		closest_waypoint_index  = (closest_waypoint_index + 1) % len(self.waypoints_cartesian)
	
	return closest_waypoint_index


    def _publish(self, next_wp_index):
	lane = Lane()
	lane.header = self.base_waypoints_msg.header
	lane.waypoints = self.base_waypoints_msg.waypoints[next_wp_index : next_wp_index + LOOKAHEAD_WPS]
	self.final_waypoints_pub.publish(lane)
	

    def _loop(self):
	rate = rospy.Rate(50)
	while not rospy.is_shutdown():
		if self.pose_msg and self.base_waypoints_msg:
			rospy.loginfo('Indide if self.pose_msg and self.base_waypoints_msg')
			next_wp_index = self._get_next_waypoint_index()
			self._publish(next_wp_index)
		rate.sleep()


    def pose_cb(self, msg):
	# The type of msg is PoseStamped.
	# Details can be seen with `rosmsg show /geometry_msgs/PoseStamped`
	self.pose_msg = msg

    def waypoints_cb(self, msg):
	# The type of msg is Lane.
	# Details can be seen with `rosmsg show /styx_msgs/Lane`
	# msg contains a list of waypoints (styx_msgs/Waypoint[])
	rospy.loginfo('Inside waypoints_cb')
        self.base_waypoints_msg = msg
	if not self.waypoints_cartesian:
		rospy.loginfo('Setting up KD tree')
		self.waypoints_cartesian =  [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in msg.waypoints]
		self.waypoints_tree = KDTree(self.waypoints_cartesian) 
		rospy.loginfo('Done setting KD Tree')

    
    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
