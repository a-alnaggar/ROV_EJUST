import rospy
from geometry_msgs.msg import Pose2D, Twist
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64
from dataclasses import dataclass
import dataclasses


@dataclasses.dataclass
class ErrorVal:
    """PID error terms"""

    e_sum: float = 0
    d_error: float = 0
    current_error: float = 0
    prev_error: float = 0


class PID:
    """PID controller Class"""

    def __init__(
        self, k_proportaiol: float, k_integral: float, k_derivative: float
    ) -> None:
        """Creates a PID controller using provided PID parameters

        Args:
            k_proportaiol (float): the proportional error constant of the desired PID
            k_integral (float): the Integral error constant of the desired PID
            k_derivative (float): the derivative error constant of the desired PID
        """
        self.k_proportaiol = k_proportaiol
        self.k_integral = k_integral
        self.k_derivative = k_derivative
        self.pid: float = 0
        self.error = ErrorVal()

    def compute(self, ref: float, measured: float) -> float:
        """Computes the PID value based on the given reference and measured output values

        Args:
            ref (float): reference signal that we desire to track
            measured (float): actual measured output of the signal

        Returns:
            float: PID value
        """
        self.error.current_error = ref - measured
        self.error.e_sum += self.error.current_error
        self.error.d_error = self.error.current_error - self.error.prev_error
        self.pid = (
            self.k_proportaiol * self.error.current_error
            + self.k_integral * self.error.e_sum
            + self.k_derivative * self.error.d_error
        )
        self.error.prev_error = self.error.current_error
        return self.pid

    def set_pid(
        self, k_proportaiol: float, k_integral: float, k_derivative: float
    ) -> None:
        """Sets the PID controller constants

        Args:
            k_proportaiol (float): the proportional error constant of the desired PID
            k_integral (float): the Integral error constant of the desired PID
            k_derivative (float): the derivative error constant of the desired PID
        """
        self.k_proportaiol = k_proportaiol
        self.k_integral = k_integral
        self.k_derivative = k_derivative


def mapFromTo(num, inMin, inMax, outMin, outMax):
  return  (float(num - inMin) / float(inMax - inMin) * (outMax - outMin)) + outMin

@dataclass
class Param:
    """tuning and utils params"""

    gamma = 1
    alpha = 1
    beta = 1
    kp_depth = 1
    ki_depth = 0
    kd_depth = 0
    # TODO: measure the floatability of the ROV
    floatability = 0
    max_pool_depth = 4
    thruster_stop = 1500
    thruster_max_forward = 1800
    thruster_max_reverse = 1200
    max_vx = 1
    min_vx = -1
    max_vy = 1
    min_vy = -1
    max_wz = 1
    min_wz = -1
    max_dz = 1
    min_dz = -1


global param
class Depth:
    def __init__(self) -> None:
        self.actual_depth = 0
global depth
depth = Depth()

global thrusters_voltages_publisher

param = Param()


def node_init():
    """Initialize motion_control node which subscribes to ROV/cmd_vel topic and the ROV/depth and publishes
    on the ROV/thrusters topic"""
    rospy.init_node("kinematic_model", anonymous=True)
    global thrusters_voltages_publisher

    thrusters_voltages_publisher = rospy.Publisher(
        "ROV/thrusters", Float32MultiArray, queue_size=10
    )

    rospy.Subscriber("ROV/cmd_vel", Twist, cmd_vel_recieved_callback)
    rospy.Subscriber("ROV/depth", Float64, depth_recieved_callback)
    rospy.spin()


def depth_recieved_callback(depth):
    """callback function for the subscriber to the ROV/depth topic

    Args:
        msg (Float64): depth sent by the the depth sensor
    """
    depth.actual_depth = depth


def cmd_vel_recieved_callback(cmd_vel):
    """callback function for the subscriber to the ROV/cmd_vel topic

    Args:
        msg (Twist): command velocity sent by the control node
    """

    w_z = cmd_vel.angular.z
    v_x = cmd_vel.linear.x
    v_y = cmd_vel.linear.y
    d_z = cmd_vel.linear.z

    w_z = mapFromTo(w_z, param.min_wz, param.max_wz, param.thruster_max_reverse, param.thruster_max_forward)
    v_x = mapFromTo(v_x, param.min_vx, param.max_vx, param.thruster_max_reverse, param.thruster_max_forward)
    v_y = mapFromTo(v_y, param.min_vy, param.max_vy, param.thruster_max_reverse, param.thruster_max_forward)
    d_z = mapFromTo(d_z, param.min_dz, param.max_dz, param.thruster_max_reverse, param.thruster_max_forward)

    # Planer Control
    phi_1 = param.gamma * v_x -param.gamma * v_y 
    phi_2 = param.gamma * v_x +param.gamma * v_y
    phi_3 = param.gamma * v_x -param.gamma * v_y
    phi_4 = param.gamma * v_x +param.gamma * v_y


    # Rotation Control
    phi_1 = param.beta * w_z
    phi_2 = -param.beta * w_z
    phi_3 = -param.beta * w_z
    phi_4 = param.beta * w_z


    planer_thrusters_list = [phi_1, phi_2, phi_3, phi_4]
    # for i in range(len(planer_thrusters_list)):
    #     if planer_thrusters_list[i] > 0:
    #         planer_thrusters_list[i] = mapFromTo(
    #             planer_thrusters_list[i],
    #             0,
    #             max_pos_vel_per_thruster,
    #             param.thruster_min,
    #             param.thruster_max_forward,
    #         )
        

    # Depth Control
    pid_depth = PID(
        k_proportaiol=param.kp_depth,
        k_integral=param.ki_depth,
        k_derivative=param.kd_depth,
    )
    pid_depth_val = (
        pid_depth.compute(ref=d_z, measured=depth.actual_depth) + param.floatability
    )

    min_pid_val = (-param.max_pool_depth / 2) * param.kp_depth
    max_pid_val = (param.max_pool_depth / 2) * param.kp_depth
    if pid_depth_val > 0:
        # TODO: Use the curve given in the data sheet of the thrusters
        phi_5 = mapFromTo(
            pid_depth_val / 2,
            min_pid_val,
            max_pid_val,
            param.thruster_stop,
            param.thruster_max_forward,
        )
        phi_6 = phi_5
    else:
        phi_5 = mapFromTo(
            pid_depth_val / 2,
            min_pid_val,
            max_pid_val,
            param.thruster_stop,
            param.thruster_max_reverse,
        )
        phi_6 = phi_5

    # phi_5 and phi_6 will be zero for now
    phi_5 = 1400
    phi_6 = 1500
    #################################

    planer_thrusters_list.append(phi_5)
    planer_thrusters_list.append(phi_6)

    # create the message instance
    thrusters_voltages = Float32MultiArray()

    # fill the message with the phi values
    thrusters_voltages.data = planer_thrusters_list

    # publish the message
    rospy.loginfo(thrusters_voltages)
    thrusters_voltages_publisher.publish(thrusters_voltages)
    rospy.sleep(0.01)


if __name__ == "__main__":
    node_init()
