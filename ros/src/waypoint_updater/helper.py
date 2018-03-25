import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math



def filter_waypoints(self_x, self_y, orientation, waypoints, limit):
    '''
    Filter the waypoints based on the current position
    '''

    selected_wps = []  #(distance, waypoint_data)

    # Loop through all the waypoints
    for wp in waypoints:
        position = wp.pose.pose.position
        x = position.x
        y = position.y

        # convert each waypoint into car's coordinate
        front, side = to_car_coord(self_x, self_y, orientation, x, y)
        distance = (front**2 + side**2) ** 0.5
        
        # Detect the forward waypoints
        # If front is positive, the waypoints is at the front
        if front >= 0:
            selected_wps.append((distance, wp))




    # Select the nearest waypoints inside the limit
    sorted_wps = sorted(selected_wps, key=lambda a: a[0])

    ### Debug use only. Uncomment this section if you want to see if the selected waypoints are correct

    # rospy.logwarn("Current Position: {}, {}. Facing {}".format(self_x, self_y, orientation))

    # chosen_wps = sorted_wps[:limit]
    # for distance, wp in chosen_wps:
    #     position = wp.pose.pose.position
    #     x = position.x
    #     y = position.y
    #     rospy.logwarn("WP {} away. At: {}, {} ".format(distance,x,y))

    
    return [ wp[1]  for wp in sorted_wps[:limit]]
    


def to_car_coord(self_x, self_y, orientation, x, y):
    '''
    Convert waypoint to car's coordinate system based on current position and orientation
    '''

    # -Shift
    shifted_x = x - self_x
    shifted_y = y - self_y

    # -Rotate
    rotated_x = shifted_x * math.cos(-orientation) - shifted_y * math.sin(-orientation)
    rotated_y = shifted_x * math.sin(-orientation) + shifted_y * math.cos(-orientation)

    return rotated_x, rotated_y
