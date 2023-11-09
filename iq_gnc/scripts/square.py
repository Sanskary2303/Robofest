#! /usr/bin/env python
# Import ROS.
import rospy
# Import the API.
from iq_gnc.py_gnc_functions import *
# To print colours (optional).
from iq_gnc.PrintColours import *
from geometry_msgs.msg import Polygon

goals = []

def polygon_callback(data):
    global goals
    # Assuming that the data received on the 'polygon' topic is of type geometry_msgs/Polygon
    # Extract the points from the polygon message.
    goals = [(point.x, point.y, point.z, 0) for point in data.points]


def main():
    # Initializing ROS node.
    rospy.init_node("drone_controller", anonymous=True)

    rospy.Subscriber("polygon", Polygon, polygon_callback)

    namespace = rospy.get_namespace()

    # Create an object for the API.
    drone = gnc_api()
    # Wait for FCU connection.
    drone.wait4connect()
    # Wait for the mode to be switched.
    drone.wait4start()

    # Create local reference frame.
    drone.initialize_local_frame()
    # Request takeoff with an altitude of 3m.
    drone.takeoff(3)
    # Specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish.
    rate = rospy.Rate(3)

    #  Specify some waypoints
    # goals = [[0, 0, 3, 0], [5, 0, 3, -90], [5, 5, 3, 0],
    #          [0, 5, 3, 90], [0, 0, 3, 180], [0, 0, 3, 0]]

    x = rospy.get_param(namespace + "x")
    y = rospy.get_param(namespace + "y")
    z = rospy.get_param(namespace + "z")
    psi = rospy.get_param(namespace + "psi")

    i = 0

    while i < len(goals):
        drone.set_destination(
            x=goals[i][0]/10 + x, y=goals[i][1]/10 + y, z=goals[i][2] + z, psi="0")
        rate.sleep()
        if drone.check_waypoint_reached():
            i += 1
    # Land after all waypoints is reached.
    drone.land()
    rospy.loginfo(CGREEN2 + "All waypoints reached landing now." + CEND)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()
