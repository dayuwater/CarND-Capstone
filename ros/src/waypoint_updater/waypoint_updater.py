#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Header

import math

# import a helper module for waypoint updater
import helper

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

LOOKAHEAD_WPS = 20 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        # TODO: Determine the message type for the two waypoints
        #rospy.Subscriber('/traffic_waypoint', None, self.traffic_cb)
        #rospy.Subscriber('/obstacle_waypoint', None, self.obstacle_cb)
        


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=10)

        # TODO: Add other member variables you need below

        # The position of the car
        self.pos_x = 0
        self.pos_y = 0
        self.orientation = 0 # facing angle

        # All Waypoints
        self.waypoint_msg = "" # The waypoint message from ROS
        self.waypoint_header = "" # Last header detected for waypoints
        self.waypoints = [] # Actual waypoint data

        # Waypoints to be published
        self.waypoints_to_publish = []

        # Sequence
        self.seq = 0
        self.start_time = rospy.Time.now()

        # TODO: Publish waypoints here
       


        rospy.spin()

    def pose_cb(self, msg):
        '''
        This is a callback function when it receives a pose message

        It is called about 20Hz ROS time (Actually 21-22Hz)

        Do we need to publish this at 50Hz? (This is for waypoints, not actuation commands)
        '''
        # TODO: Implement
        
        # Decompose message
        header = msg.header
        pose = msg.pose

        pos_x = pose.position.x
        pos_y = pose.position.y
        pos_z = pose.position.z

        ori_z = pose.orientation.z
        ori_w = pose.orientation.w

        logmsg = "The car is at ({}, {}), facing({})".format(pos_x, pos_y, ori_z)
        #rospy.logwarn(logmsg)

        rate = rospy.Rate(50)

        self.pos_x = pos_x
        self.pos_y = pos_y
        self.orientation = ori_z

        # Determine the waypoints to be published
        self.waypoints_to_publish = helper.filter_waypoints(self.pos_x, self.pos_y, self.orientation, self.waypoints, LOOKAHEAD_WPS)
        # rospy.logwarn(self.waypoints_to_publish)

        msg = Lane()

        # Compose the header
        header = Header()
        header.seq = self.seq
        header.stamp = rospy.Time.now()
        header.frame_id = "/world"

        msg.header = header

        # msg.header = self.waypoint_header
        msg.waypoints = self.waypoints_to_publish

        # rospy.logwarn(msg)
        self.final_waypoints_pub.publish(msg)

        self.seq += 1
        # rospy.logwarn(self.seq)
        rate.sleep()

        


        #pass


        

    def waypoints_cb(self, waypoints):
        '''
        Callback function for waypoint loader

        It seems that this function is only called when the program starts
        '''

        # TODO: Implement
        self.waypoint_msg = waypoints

        # Parse the waypoints
        self.waypoint_header = waypoints.header
        self.waypoints = waypoints.waypoints

        rospy.logwarn("111")

        #pass

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
