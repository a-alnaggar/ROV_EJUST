#!/usr/bin/env python3


# import rospy
# from geometry_msgs.msg import Twist

# # Create a new Twist message
# twist_msg = Twist()

# # Set the linear and angular velocities
# twist_msg.linear.x = 0.5
# twist_msg.angular.z = 0.2

# # Publish the Twist message to the /cmd_vel topic
# rospy.init_node('twist_publisher')
# pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
# pub.publish(twist_msg)

import rospy

from geometry_msgs.msg import Twist
from pysticks import get_controller


controller = get_controller()

def talker():
    pub = rospy.Publisher("joystick", Twist, queue_size=10)
    rospy.init_node("talker", anonymous=True)
    rate = rospy.Rate(10)
    twist_msg = Twist()
    while not rospy.is_shutdown():
        
        controller.update()
        
        twist_msg.linear.x = controller.getPitch()
        twist_msg.linear.y = controller.getRoll()
        twist_msg.linear.z = controller.getThrottle()
        twist_msg.angular.z = controller.getYaw()
        
        rospy.loginfo("Throttle: %+2.2f   Roll: %+2.2f   Pitch: %+2.2f   Yaw: %+2.2f   Aux: %+2.2f"
            % (
                controller.getThrottle(),
                controller.getRoll(),
                controller.getPitch(),
                controller.getYaw(),
                controller.getAux(),
            ))
        pub.publish(twist_msg)
        rate.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
