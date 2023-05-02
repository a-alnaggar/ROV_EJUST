import rospy
from geometry_msgs.msg import Pose2D, Twist
from dataclasses import dataclass

# from pid import PID

global cmd_vel_publisher


@dataclass
class Param:
    """tuning and utils params"""

    x_const: float = 1
    y_const: float = 1
    theta_const: float = 1
    z_const: float = 1


global param
param = Param()


def node_init():
    """Initialize motion_control node which subscribes to ROV/ctrl_sig topic and publishes
    on the ROV/cmd_vel topic during manual_control"""
    rospy.init_node("motion_control", anonymous=True)
    global cmd_vel_publisher
    cmd_vel_publisher = rospy.Publisher("ROV/cmd_vel", Twist, queue_size=10)
    rospy.Subscriber("ROV/ctrl_sig", Twist, ctrl_sig_recieved_callback)
    rospy.spin()


def ctrl_sig_recieved_callback(msg):
    """callback function for the subscriber to the ROV/line_features topic

    Args:
        msg (Pose2D): line features vector as a Pose2D msg [x width theta]
    """
    x = msg.linear.x
    y = msg.linear.y
    z = msg.linear.z
    theta = msg.angular.z

    v_x = param.x_const * x  # desired v_x
    v_y = param.y_const * y  # desired v_y
    d_z = param.z_const * z  # desired d_z
    w_z = param.theta_const * theta  # desired w_z

    vel = Twist()
    vel.angular.x = 0
    vel.angular.y = 0
    vel.angular.z = v_y  # desired w_z
    vel.linear.x = v_x  # desired v_x
    vel.linear.y = w_z  # desired v_y
    vel.linear.z = d_z  # desired d_z

    rospy.loginfo(
        f"v_x: {vel.linear.x}, v_y: {vel.linear.y}, v_z: {vel.linear.z}, w_z: {vel.angular.z}"
    )
    cmd_vel_publisher.publish(vel)
    rospy.sleep(0.01)

if __name__ == "__main__":
    node_init()
