import rospy
from geometry_msgs.msg import Twist
import numpy as np

class Fetch:
    """
    Core class for controlling the Fetch robot.
    """
    def __init__(self):
        """Initialize the Fetch robot interface."""
        try:
            rospy.init_node('fetch_controller', anonymous=True)
        except rospy.exceptions.ROSException:
            print("Node has already been initialized, do nothing")
        
        # Publisher for base movement commands
        self._base_publisher = rospy.Publisher(
            "/base_controller/command", 
            Twist, 
            queue_size=2
        )
        
        # Control parameters
        self.control_rate = 10
        self.rate = rospy.Rate(self.control_rate)
        
        # Movement limits
        self.max_linear_speed = 0.5  # m/s
        self.max_angular_speed = 1.0  # rad/s

    def move_base(self, linear_x, angular_z):
        """
        Move the robot base with specified linear and angular velocities.
        
        Args:
            linear_x (float): Forward/backward velocity (-1.0 to 1.0)
            angular_z (float): Rotational velocity (-1.0 to 1.0)
        """
        # Clip velocities to safe ranges
        linear_x = np.clip(linear_x, -1.0, 1.0) * self.max_linear_speed
        angular_z = np.clip(angular_z, -1.0, 1.0) * self.max_angular_speed
        
        # Create and publish movement command
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        
        self._base_publisher.publish(twist)
    
    def stop_base(self):
        """Stop all base movement."""
        twist = Twist()
        self._base_publisher.publish(twist)