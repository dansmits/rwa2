#!/usr/bin/env python

# Import statements for modules needed outside program
import math
import rospy
import tf
import sys
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

rospy.init_node("move_robot")  # Initialize the ROS node for the process
# Create instance of publisher to send move commands to topic cmd_vel
pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)
velocity_msg = Twist()  # Global variable for angular and linear velocity messages
rate = rospy.Rate(4)  # we publish the velocity at 4 Hz (4 times per second)
# Create transform instance to get pose of Robot
tf_listener = tf.TransformListener()
odom_frame = 'odom'
base_frame = 'base_footprint'
k_h_gain = 1
k_v_gain = 1
# Initialize global variables for /Scan topic listener to share data with other functions
front_laser = 1.0
right_laser = 1.0
left_laser = 1.0
diagonal_laser_right = 1.0
diagonal_laser_left = 1.0
diagonal_laser_left2 = 1.0
diagonal_laser_right2 = 1.0

try:
    tf_listener.waitForTransform(odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1))
    base_frame = 'base_footprint'
except (tf.Exception, tf.ConnectivityException, tf.LookupException):
    try:
        tf_listener.waitForTransform(odom_frame, 'base_link', rospy.Time(), rospy.Duration(1))
        base_frame = 'base_link'
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
        rospy.signal_shutdown("tf Exception")


def get_odom_data():
    """Return position and rotation of robot using euler transform.

    Parameters: None

    Returns:
        Point(*trans): Position of robot
        rotation[2]: Rotation of robot
    """
    try:
        (trans, rot) = tf_listener.lookupTransform(odom_frame, base_frame, rospy.Time(0))
        rotation = euler_from_quaternion(rot)
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        rospy.loginfo("TF Exception")
        return
    return Point(*trans), rotation[2]


def get_angle_to_goal(goal_x, goal_y):
    """Returns angle from robot to goal

    Parameters:
         goal_x (int): x coordinate to goal
         goal_y (int): y coordinate to goal
    Returns:
        angle_to_goal (float): degrees from robot to goal
    """
    (position, rotation) = get_odom_data()  # retrieve current pose of robot
    x_start = position.x  # starting x coordinate of robot
    y_start = position.y  # starting y coordinate of robot
    angle_to_goal = math.atan2(goal_y - y_start, goal_x - x_start)  # Calculate angle to goal
    # The domain of arctan(x) is (-inf, inf)
    # We would like to restrict the domain to (0, 2pi)
    if angle_to_goal < -math.pi / 4 or angle_to_goal > math.pi / 4:
        if 0 > goal_y > y_start:
            angle_to_goal = -2 * math.pi + angle_to_goal
        elif 0 <= goal_y < y_start:
            angle_to_goal = 2 * math.pi + angle_to_goal
    return angle_to_goal


def get_distance_to_goal(goal_x, goal_y):
    """Returns distance of robot to goal

        Parameters:
             goal_x (int): x coordinate to goal
             goal_y (int): y coordinate to goal
        Returns:
            distance_to_goal (float): Meters from robot to goal
        """
    (position, rotation) = get_odom_data()  # retrieve current pose of robot
    distance_to_goal = math.sqrt((goal_x - position.x) ** 2 + (goal_y - position.y) ** 2)
    return distance_to_goal


def turn_to_goal(goal_x, goal_y):
    """Turn robot towards goal before moving

    Parameters:
         goal_x (int): x coordinate to goal
         goal_y (int): y coordinate to goal
    Returns:
        None
    """
    (position, rotation) = get_odom_data()  # retrieve current pose of robot
    rospy.logwarn('Robot is turning towards goal')
    velocity_msg.linear.x = 0.0  # Stop robot at current goal
    velocity_msg.angular.z = k_v_gain * get_angle_to_goal(goal_x, goal_y) - rotation

    while abs(get_angle_to_goal(goal_x, goal_y) - rotation) > 0.1:
        pub.publish(velocity_msg)  # publish angular velocity to topic /cmd_vel
        (position, rotation) = get_odom_data()  # retrieve current pose of robot


def go_to_goal():
    """Initiate navigation of robot to goal

    Parameters:

    Returns:
        None
    """
    goals = [(1, -2), (-1, 2), (2, 0), (-1, -2), (1, 2)]  # Set coordinates for robot to travel to
    last_rotation = 0

    for coordinates in goals:  # initiate navigation to each goal
        goal_x, goal_y = coordinates
        rospy.logwarn('Robot is traveling to goal: {}'.format(coordinates))
        rospy.logwarn('Robot is {:.2f} meters from goal'.format(get_distance_to_goal(goal_x, goal_y)))

        # Turn towards Goal
        turn_to_goal(goal_x, goal_y)

        while get_distance_to_goal(goal_x, goal_y) > 0.05:
            (position, rotation) = get_odom_data()  # retrieve current pose of robot

            if last_rotation > math.pi - 0.1 and rotation <= 0:
                rotation = 2 * math.pi + rotation
            elif last_rotation < -math.pi + 0.1 and rotation > 0:
                rotation = -2 * math.pi + rotation

            # Adjust robot angular velocity to angle to goal
            velocity_msg.angular.z = k_v_gain * get_angle_to_goal(goal_x, goal_y) - rotation

            # Set speed and distance of robot to values that are slow enough to allow collision avoidance
            velocity_msg.linear.x = min(0.1, 0.5)

            if velocity_msg.angular.z > 0:
                velocity_msg.angular.z = min(velocity_msg.angular.z, 1.5)
            else:
                velocity_msg.angular.z = max(velocity_msg.angular.z, -1.5)

            collision_avoidance()  # Call function to checking for approaching obstacles

            last_rotation = rotation
            pub.publish(velocity_msg)  # Publish angular & linear velocity to /cmd_vel topic
            rate.sleep()  # Pause messages while robot executes command

        rospy.logwarn("Robot has successfully reached goal: {}".format(coordinates))
        velocity_msg.linear.x = 0.0
        velocity_msg.angular.z = 0.0
        pub.publish(velocity_msg)

    rospy.logwarn("Robot has reached final goal, shutdown initiated")
    rospy.signal_shutdown("Robot has reached final Goal!")


def collision_avoidance():
    """Uses lidar sensors on robot to adjust course to goal to avoid obstacles

    Parameters:

    Returns:
        None
    """
    if front_laser < .45:
        # Do collision avoidance
        rospy.logwarn("Warning, approaching obstacle detected in front of Robot, turning right")
        rospy.loginfo("Distance from obstacle bearing 000: {:.2f} meters".format(front_laser))
        velocity_msg.angular.z = -0.5  # Turn robot right to avoid obstacle
    elif right_laser < .2 or diagonal_laser_right < .45 or diagonal_laser_right2 < .45:
        # Do collision avoidance
        rospy.logwarn("Warning, approaching obstacle detected on right side of Robot, turning left")
        rospy.loginfo("Distance from obstacle bearing 270: {:.2f} meters".format(right_laser))
        rospy.loginfo("Distance from obstacle bearing 315: {:.2f} meters".format(diagonal_laser_right))
        rospy.loginfo("Distance from obstacle bearing 338: {:.2f} meters".format(diagonal_laser_right2))
        velocity_msg.angular.z = 0.5  # Turn robot left to avoid obstacle
    elif left_laser < .2 or diagonal_laser_left < .45 or diagonal_laser_left2 < .45:
        # Do collision avoidance
        rospy.logwarn("Warning, approaching obstacle detected on left side of Robot, turning right")
        rospy.loginfo("Distance from obstacle bearing 090: {:.2f} meters".format(left_laser))
        rospy.loginfo("Distance from obstacle bearing 045: {:.2f} meters".format(diagonal_laser_left))
        rospy.loginfo("Distance from obstacle bearing 022: {:.2f} meters".format(diagonal_laser_left2))
        velocity_msg.angular.z = -0.5  # Turn robot right to avoid obstacle

    pub.publish(velocity_msg)  # Publish obstacle avoidance changes to topic /cmd_vel


def sensor_callback(msg):
    """Sensor callback function for read_scan() subscriber of the /Scan topic

    Parameters:
         msg (int): specifies the desired laser to retrieve info based on angular degree from robot
    Returns:
         None
    """
    global front_laser, left_laser, right_laser, diagonal_laser_left, diagonal_laser_left2, diagonal_laser_right, \
        diagonal_laser_right2

    # Update global variables for robot laser
    front_laser = msg.ranges[0]
    diagonal_laser_right = msg.ranges[315]
    right_laser = msg.ranges[270]
    left_laser = msg.ranges[90]
    diagonal_laser_left = msg.ranges[45]
    diagonal_laser_left2 = msg.ranges[22]
    diagonal_laser_right2 = msg.ranges[338]


def read_scan():
    """Subscriber for the /Scan Topic

    Parameters:

    Returns:
        None
    """
    rospy.Subscriber("scan", LaserScan, sensor_callback)


if __name__ == "__main__":
    while not rospy.is_shutdown():
        read_scan()
        go_to_goal()
    sys.exit()
