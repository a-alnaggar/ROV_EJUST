#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Twist, Vector3
from pysticks import get_controller


controller = get_controller()

def talker():
    pub = rospy.Publisher("ROV/ctrl_sig", Twist, queue_size=10)
    pid_cal_pub = rospy.Publisher("ROV/PID",Vector3,queue_size=10)
    apg_cal_pub = rospy.Publisher("ROV/params",Vector3,queue_size=10)
    op_cal_pub = rospy.Publisher("ROV/op",Vector3,queue_size=10)
    rospy.init_node("talker", anonymous=True)
    rate = rospy.Rate(10)
    twist_msg = Twist()

    pid_cal_msg = Vector3()
    apg_cal_msg = Vector3()
    op_cal_msg = Vector3()

    while not rospy.is_shutdown():
        
        controller.update()
        
        twist_msg.linear.x = controller.getPitch()
        twist_msg.linear.y = controller.getRoll()
        twist_msg.linear.z = controller.getThrottle()
        twist_msg.angular.z = controller.getYaw()

        #pid_val from ros topic Pub from terminal




        
        rospy.loginfo("Throttle: %+2.2f   Roll: %+2.2f   Pitch: %+2.2f   Yaw: %+2.2f   Aux: %+2.2f"
            % (
                controller.getThrottle(),
                controller.getRoll(),
                controller.getPitch(),
                controller.getYaw(),
                controller.getAux(),
            ))
        rospy.loginfo("")
        pub.publish(twist_msg)
        pid_cal_pub.publish(pid_cal_msg)
        apg_cal_pub.publish(apg_cal_msg)
        op_cal_pub.publish(op_cal_msg)
        rate.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
