#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from pysticks import get_controller


controller = get_controller()

def talker():
    pub = rospy.Publisher("ROV/joystick", Twist, queue_size=10)
    gripper = rospy.Publisher("ROV/gripper",Bool,queue_size=1)

    rospy.init_node("Joystick", anonymous=True)
    rate = rospy.Rate(10)
    
    twist_msg = Twist()
    grip_msg = Bool()

    while not rospy.is_shutdown():
        
        controller.update()
        aimball_x = controller.getAimball()[0]
        aimball_y = controller.getAimball()[1]
        
        
        # Get depth value readings
        controller.depthUpFine()
        controller.depthUpCoarse()
        controller.depthDownFine()
        controller.depthDownCoarse()
        
        twist_msg.linear.x = controller.getPitch() if not aimball_x else 0
        twist_msg.linear.y = controller.getRoll() if not aimball_y else 0
        twist_msg.linear.z = controller.depth
        twist_msg.angular.z = controller.getYaw() if not aimball_x and not aimball_y else 0
        
        grip_msg.data = bool(controller.getTrigger() + 1)
        
        rospy.loginfo("Throttle: %+2.2f   Roll: %+2.2f   Pitch: %+2.2f   Yaw: %+2.2f   Aux: %+2.2f"
            % (
                controller.depth,
                controller.getRoll(),
                controller.getPitch(),
                controller.getYaw(),
                bool(controller.getTrigger() + 1),
            ))
        
        gripper.publish(grip_msg)
        

        if controller.stopAll():
            twist_msg = Twist()
            
        pub.publish(twist_msg)
        rate.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass