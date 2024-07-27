#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray

def publish_waypoints():
    rospy.init_node('waypoint_publisher', anonymous=True)
    waypoints_pub = rospy.Publisher('/waypoints', Float64MultiArray, queue_size=10)

    waypoints = [
        0.0, 0.0, 0.8, 0.0,  # Waypoint 1
        0.0, -2.0, 2.0, 180.0,  # Waypoint 2
        1.0, -1.0, 2.0, 0.0,  # Waypoint 3
        #0.0, -2.0, 2.0, 180.0,   # Waypoint 4
        0.0, 0.0, 2.0, 0.0      # Waypoint 5
        
        
    ]

    msg = Float64MultiArray(data=waypoints)

    rate = rospy.Rate(0.2)  # Publish once every 5 seconds
    while not rospy.is_shutdown():
        waypoints_pub.publish(msg)
        #rospy.loginfo("Published waypoints: %s", waypoints)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_waypoints()
    except rospy.ROSInterruptException:
        pass
